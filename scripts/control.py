from PIL import Image
import numpy
import os
from ament_index_python.packages import get_package_share_directory

pkg_path = os.path.join(get_package_share_directory('Penguin'))
pgm_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.pgm')
image = Image.open(pgm_file)
image_array = numpy.array(image)
print(f"Image size: {image.size}")
print(f"Image mode: {image.mode}")
print("Pixel data:")
print(image_array)