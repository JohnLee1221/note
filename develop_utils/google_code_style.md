## eunm

**枚举**的命名应当和 `常量` 或 `宏` 一致: `kEnumName` 或是 `ENUM_NAME`

单独的**枚举**值应该优先采用 `常量` 的命名方式. 但 `宏` 方式的命名也可以接受. 枚举名 `UrlTableErrors` (以及 `AlternateUrlTableErrors`) 是类型, 所以要用大小写混合的方式.

```
enum UrlTableErrors {
    kOK = 0,
    kErrorOutOfMemory,
    kErrorMalformedInput,
};
enum AlternateUrlTableErrors {
    OK = 0,
    OUT_OF_MEMORY = 1,
    MALFORMED_INPUT = 2,
};
```



## 构造函数初始化列表

```c++
// 如果所有变量能放在同一行:
MyClass::MyClass(int var) : some_var_(var) {
  DoSomething();
}

// 如果不能放在同一行,
// 必须置于冒号后, 并缩进 4 个空格
MyClass::MyClass(int var)
    : some_var_(var), some_other_var_(var + 1) {
  DoSomething();
}

// 如果初始化列表需要置于多行, 将每一个成员放在单独的一行
// 并逐行对齐
MyClass::MyClass(int var)
    : some_var_(var),             // 4 space indent
      some_other_var_(var + 1) {  // lined up
  DoSomething();
}

// 右大括号 } 可以和左大括号 { 放在同一行
// 如果这样做合适的话
MyClass::MyClass(int var)
    : some_var_(var) {}
```



## lambda表达式

```c++
int x = 0;
auto add_to_x = [&x](int n) { x += n; };
```



```c++
auto f = 
    [this](cosnt std::vector<int>& vec) {
      this->foo(vec);
	};

std::function<void(const std::shared_ptr<int>&)> func;
func = [this](const std::shared_ptr<int>& ptr) {
  this->Test(ptr);
};
```



```c++
// 长lambda
std::set<int> blacklist = {7, 8, 9};
std::vector<int> digits = {3, 4, 5, 8, 1};
digits.erase(std::remove_if(digits.begin(), digits.end(), [&blacklist](int i) {
			   return backlist.find(i) != blacklist.end();
			 }),
             degits.end());
```



## switch

```c++
switch (section.type) {
  case SectionType::SECTION_INDEX: {
    file_reader_->SkipSection(section.size);
    reach_end_ = true;
    break;
  }
  case SectionType::SECTION_CHANNEL: {
    ADEBUG << "Read channel section of size: " << section.size;
    Channel channel;
    if (!file_reader_->ReadSection<Channel>(section.size, &channel)) {
      AERROR << "Failed to read channel section.";
      return false;
    }
    break;
  }
  case SectionType::SECTION_CHUNK_HEADER: {
    ADEBUG << "Read chunk header section of size: " << section.size;
    ChunkHeader header;
    if (!file_reader_->ReadSection<ChunkHeader>(section.size, &header)) {
      AERROR << "Failed to read chunk header section.";
      return false;
    }
    if (header.end_time() < begin_time) {
      skip_next_chunk_body = true;
    }
    if (header.begin_time() > end_time) {
      return false;
    }
    break;
  }
  case SectionType::SECTION_CHUNK_BODY: {
    if (skip_next_chunk_body) {
      file_reader_->SkipSection(section.size);
      skip_next_chunk_body = false;
      break;
    }

    chunk_.reset(new ChunkBody());
    if (!file_reader_->ReadSection<ChunkBody>(section.size, chunk_.get())) {
      AERROR << "Failed to read chunk body section.";
      return false;
    }
    return true;
  }
  default: {
    AERROR << "Invalid section, type: " << section.type
            << ", size: " << section.size;
    return false;
  }
}
```

