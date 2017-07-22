# tk3\_recognition

## node\_person\_recogniton

1. feed in an image
2. return some candidate face bboxes with essential infomation

### service definition

```plain
sensor_msgs/Image face_image
---
int32 face_amount
int32[] face_gender
geometry_msgs/Polygon[] face_pos   # just diagonal
```

## node\_voice\_recognition

1. Always listen at microphone
2. return the sentence

### topic definition

```plain
string (anonymous)
```

