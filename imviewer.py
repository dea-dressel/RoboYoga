from skimage.viewer import ImageViewer
from skimage.io import imread

img = imread('poses.png') #path to IMG
view = ImageViewer(img)
view.show()