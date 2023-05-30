from PIL import Image
import redis

# creating a object
im = Image.open(r"poses.png")

pool = redis.ConnectionPool(host='localhost', port=6379, db=0)
redis = redis.Redis(connection_pool=pool)
redis.set('initialized', False)
initialize = input(
    "Stand in the starting position of the kinect and press ENTER when ready")
redis.set('initialized', True)

while True:
    im.show()
    pose = input("Select a pose (input a number 1-5): ")
    print(pose)
    redis.set('pose::selection', pose)
    value = redis.get('pose::selection')
    print(value)
