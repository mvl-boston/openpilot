import sys
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'
BIG_WARP_SIZES = ('1344x760', '1928x1208')
LFS_POINTER_HEADER = b'version https://git-lfs.github.com/spec/v1'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def big_model_paths() -> list[Path]:
  # everything stock modeld loads to run on chestnut
  return [modeld_pkl_path(chestnut=True)] + [MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl' for size in BIG_WARP_SIZES]

def is_lfs_pointer(path: Path) -> bool:
  try:
    with open(path, 'rb') as f:
      return f.read(len(LFS_POINTER_HEADER)) == LFS_POINTER_HEADER
  except OSError:
    return False

def load_oob(path, chestnut=False):
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not chestnut_present():
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

def chestnut_compiled() -> bool:
  # an unfetched LFS pointer is not a loadable model
  return all(path.is_file() and not is_lfs_pointer(path) for path in big_model_paths())
