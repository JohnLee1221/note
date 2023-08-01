# Protobuf

### FindFieldByName(...)



### bool ParseFromString(...);



### cpp_type()

```c++
  enum CppType {
    CPPTYPE_INT32       = 1,     // TYPE_INT32, TYPE_SINT32, TYPE_SFIXED32
    CPPTYPE_INT64       = 2,     // TYPE_INT64, TYPE_SINT64, TYPE_SFIXED64
    CPPTYPE_UINT32      = 3,     // TYPE_UINT32, TYPE_FIXED32
    CPPTYPE_UINT64      = 4,     // TYPE_UINT64, TYPE_FIXED64
    CPPTYPE_DOUBLE      = 5,     // TYPE_DOUBLE
    CPPTYPE_FLOAT       = 6,     // TYPE_FLOAT
    CPPTYPE_BOOL        = 7,     // TYPE_BOOL
    CPPTYPE_ENUM        = 8,     // TYPE_ENUM
    CPPTYPE_STRING      = 9,     // TYPE_STRING, TYPE_BYTES
    CPPTYPE_MESSAGE     = 10,    // TYPE_MESSAGE, TYPE_GROUP
    MAX_CPPTYPE         = 10,    // Constant useful for defining lookup tables
                                 // indexed by CppType.
  };
```