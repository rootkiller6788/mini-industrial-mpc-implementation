
import os, json, sys

BASE = sys.argv[1] if len(sys.argv) > 1 else '.'
target = sys.argv[2] if len(sys.argv) > 2 else 'all'

def build_file(filepath, includes, functions):
    out = []
    w = out.append
    for inc in includes:
        w(f'#include {inc}')
    w('')
    for func in functions:
        name, ret_type, params, doc, body = func
        for d in doc:
            w(d)
        w(f'{ret_type} {name}({params})')
        w('{')
        for b in body:
            w(f'    {b}')
        w('}')
        w('')
    path = os.path.join(BASE, filepath)
    with open(path, 'w') as f:
        f.write('
'.join(out) + '
')
    return len(out)

# Function definitions will be appended by the build process
defs = {}
