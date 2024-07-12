# pseyestereovisiondemo
A basic distance measurement using apriltags and two ps3eye cameras
# Dependacies
- [pseyepy](https://github.com/bensondaled/pseyepy)
- apriltag
- opencv

# Adjusting some values
```
aprltagTYP = "tag25h9"
fps = 30 #this is obvious
iro = False #Boolean True is Yes color
gain = 50 #max of 63 higher values increases gain at the cost of noise
exp = 63 #max of 63 higher values increases brightness at the cost of motion blur
#################################################
fov = 44.5 #field of view of both cameras
diff = 8.5725 #the distance between the two cameras from distance
#################################################
```
Please note that the diff is likely the only variable you need to change to get the correct values. 




