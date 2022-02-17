# Python - Threads and Processes

- [Python](#python)
  - [Collections](#collections)
    - [Counter](#counter)
    - [namedtuple](#namedtuple)
  - [Logging](#logging)
  - [Random](#random)
    - [random: reproducible randoms](#random-reproducible-randoms)
    - [secrets: real randoms](#secrets-real-randoms)

A ***process*** is an instance of a program, menawhile a ***thread*** is an entity in a process, then a process could have more threads whitin.


|     | Process                                | Thread                                          |
| --- | -------------------------------------- | ----------------------------------------------- |
| x   | take advantage of multi-CPUs and cores |                                                 |
| x   | separate memory space                  | shared memory space (can raise race conditions) |
| x   | CPU-bound processing                   | I/O-bound tasks                                 |
| x   | one GIL for each process               | is limited by GIL (one thread at time)          |
| x   | heavyweight                            | lightweight                                     |
| x   | more memory                            |                                                 |
| x   | inter-process communication            |                                                 |
| x   | easy to kill                           | no killable                                     |

Possible solutions to avoid the GIL (Global Interpreter Lock) are:
- using multi-processing
- use different, free-threaded Python implementation
- use Python as wrapper for third-party libraries (C/C++)


## Multi-processing

```python
from multiprocessing import Process

# custom function
def square_num():
  for i in range(100):
    i * i

# define processes
processes = []
num_processes = os.cpu_count()

# create processes
for x in range(num_processes):
  p = Process(target=square_num)
  processes.append(p)

# start each process
for p in processes:
  p.start()

# join
for p in processes:
  p.join()
```


## Multi-threading

```python
from threading import Thread

# custom function
def square_num():
  for i in range(100):
    i * i

# define threads
thread = []
num_threads = 10

# create threads
for x in range(num_threads):
  t = Thread(target=square_num)
  threads.append(t)

# start each process
for t in threads:
  t.start()

# join
for t in threads:
  t.join()


```

