```cpp
auto buffer = (uint8_t *)msg.data.data();
cv::Mat frame(msg.rows, msg.cols, msg.type, buffer);
```

```cpp
uint32 rows
uint32 cols
uint8 channels
uint8 type
uint32 offset
uint32 size
uint64 count
uint64 timestamp

uint32 MAX_SIZE=6291456
char[6291456] data
```

export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml

