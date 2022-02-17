# Python - Common Modules

- [Python - Common Modules](#python---common-modules)
  - [Collections](#collections)
    - [Counter](#counter)
    - [namedtuple](#namedtuple)
  - [Logging](#logging)
  - [Random](#random)
    - [random: reproducible randoms](#random-reproducible-randoms)
    - [secrets: real randoms](#secrets-real-randoms)
  - [Parsing parameters](#parsing-parameters)
    - [Needed arguments](#needed-arguments)
    - [Optional arguments](#optional-arguments)




## Collections

### Counter
```python
from collections import Counter
counter = Counter(str_)    # return a dictionary with keys the characters and values their duplicates

# methods
counter.most_common(x)      # return only the x higher pairs
counter.elements()          # create an iterator
```

### namedtuple
```python
from collections import namedtuple

Tuple = namedtuple('Tuple', 'field_1, ..., field_n')
t = Tuple('value_1, ..., value_n')    # assign values to the named fields
```


## Logging

```python
import logging

loggingin.basicConfig(level, format, datefmt)

logging.debug('Message')
logging.info('Message')
logging.warning('Message')
logging.error('Message')
logging.critical('Message')
```


## Random

### random: reproducible randoms

```python
import random

random.seed(int_)    # set the seed

value = random.random()   # random num from 0 to 1
value = random.randint(start, end)      # random integer
value = random.uniform(start, end)      # random unform distribution
value = random.normalvariate(mean, std_dev)      # random with normal distribution

value = random.choice(list_)  # pick a random element from a list
value = random.choices(list_, k=n)  # pick n random elements from a list

```


### secrets: real randoms

```python
import secrets

value = secrets.randbelow(upper_bound)  # return a number from 0 to upper_bound
value = secrets.randbits(n_bits)      # return a int from 0 to a int of max n_bits
value = secrets.choice(list_)         # return a random element of the list
```


## Parsing parameters


### Needed arguments

```sh
pyhton3 executable.py param_1 ... param_N
```

```python
import sys

sys.argv        # list of arguments parsed
sys.argv[0]     # name of the executable
sys.argv[1]     # param_1
...
sys.argv[N]     # param_N
```

### Optional arguments

```sh
pyhton3 executable.py -tag_1 arg_1 ... -tag_N arg_N
```

```python
import sys
import getopt

opts, args = getopt.getopt(sys.argv[1:], "tag_1:tag_2:...:tag_N")

```

```python
import sys
import argparse


```