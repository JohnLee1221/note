## docker安装

https://blog.csdn.net/m0_60827485/article/details/125197867?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168895864116800188595961%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168895864116800188595961&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-125197867-null-null.142^v88^insert_down38v5,239^v2^insert_chatgpt&utm_term=ubuntu%2020docker%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187



## 彻底卸载docker

https://m.php.cn/faq/520283.html



## ssh到docker

```shell
stu@XXX:~$ docker run -it --gpus all --name ymz --shm-size="2g" -p 8088:99 -v /data/ymz:/data/ymz pytorch/pytorch:1.9.1-cuda11.1-cudnn8-devel /bin/bash
```

