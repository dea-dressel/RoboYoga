from PIL import Image
import redis
 
# creating a object
im = Image.open(r"poses.png")
 
im.show()

pool = redis.ConnectionPool(host='localhost', port=6379, db=0)
redis = redis.Redis(connection_pool=pool)
while True:
    pose = input("Select a pose (input a number 1-5): ")
    print(pose)
    redis.set('PoseSelection', pose)
    value = redis.get('PoseSelection')
    print(value)
