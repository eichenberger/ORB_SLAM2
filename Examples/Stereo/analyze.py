import sys
import io
import re

count = 0
exec_time_t = 0.0

with io.open(sys.argv[1], "r") as f:
    for l in f:
        name = re.sub(r'.*my_provider:([^:]*):.*\n', r'\1', l)
        exec_time = re.sub(r'.*\+([^\)]*).*\n', r'\1', l)
        if name == sys.argv[2]:
            exec_time_t = exec_time_t + float(exec_time)
            count = count + 1

print("total exec time: " + str(exec_time_t/count))




