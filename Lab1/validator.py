import sys

def validate(out_file, tc_file):
    with open(out_file) as f: lines = f.read().strip().split('\n')
    with open(tc_file) as f: tc_lines = f.read().strip().split('\n')

    top_pins = list(map(int, tc_lines[1].split()))
    bot_pins = list(map(int, tc_lines[2].split()))

    grid_h = {}
    grid_v = {}
    
    current_net = 0
    max_y = 0
    
    net_segments = []

    for line in lines:
        parts = line.split()
        if not parts: continue
        if parts[0] == '.begin':
            current_net = int(parts[1])
        elif parts[0] == '.H':
            x1, y, x2 = map(int, parts[1:])
            max_y = max(max_y, y)
            for x in range(min(x1, x2), max(x1, x2) + 1): # actually H segments are x1 to x2 interval
                if y == 0 or y > len(top_pins)+1000: pass # wait, top row is W+1
                # checking overlap:
            net_segments.append(('H', current_net, x1, x2, y))
        elif parts[0] == '.V':
            x, y1, y2 = map(int, parts[1:])
            max_y = max(max_y, max(y1, y2))
            net_segments.append(('V', current_net, x, y1, y2))
            
    # basic check
    print(f"File: {out_file}, Max Y: {max_y}")

    # short check
    for ty, n, a, b, c in net_segments:
        if ty == 'H':
            x1, x2, y = a, b, c
            for x in range(x1, x2): # segment is [x, x+1]
                if (x, y) in grid_h and grid_h[(x, y)] != n:
                    print(f"H Short at y={y}, x={x}-{x+1} between {n} and {grid_h[(x, y)]}")
                grid_h[(x, y)] = n
        if ty == 'V':
            x, y1, y2 = a, b, c
            for y in range(y1, y2):
                if (x, y) in grid_v and grid_v[(x, y)] != n:
                    print(f"V Short at x={x}, y={y}-{y+1} between {n} and {grid_v[(x, y)]}")
                grid_v[(x, y)] = n
                
validate("out1_TA.txt", "testcase/testcase1.txt")
try:
    validate("out2_TA.txt", "testcase/testcase2.txt")
except: pass
