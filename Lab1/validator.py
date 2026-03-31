import sys

def validate(out_file, tc_file):
    with open(out_file) as f: lines = f.read().strip().split('\n')
    with open(tc_file) as f: tc_lines = f.read().strip().split('\n')

    top_pins = list(map(int, tc_lines[1].split()))
    bot_pins = list(map(int, tc_lines[2].split()))

    grid_h = {}
    grid_v = {}
    
    current_net = 0
    net_segments = []

    for line in lines:
        parts = line.split()
        if not parts: continue
        if parts[0] == '.begin':
            current_net = int(parts[1])
        elif parts[0] == '.H':
            x1, y, x2 = map(int, parts[1:])
            net_segments.append(('H', current_net, x1, x2, y))
        elif parts[0] == '.V':
            x, y1, y2 = map(int, parts[1:])
            net_segments.append(('V', current_net, x, y1, y2))
            
    # short check
    for ty, n, a, b, c in net_segments:
        if ty == 'H':
            x1, x2, y = a, b, c
            for x in range(min(x1, x2), max(x1, x2)): # segment is [x, x+1]
                if (x, y) in grid_h and grid_h[(x, y)] != n:
                    print(f"H Short at y={y}, x={x}-{x+1} between Net {n} and Net {grid_h[(x, y)]}")
                grid_h[(x, y)] = n
        if ty == 'V':
            x, y1, y2 = a, b, c
            for y in range(min(y1, y2), max(y1, y2)):
                if (x, y) in grid_v and grid_v[(x, y)] != n:
                    print(f"V Short at x={x}, y={y}-{y+1} between Net {n} and Net {grid_v[(x, y)]}")
                grid_v[(x, y)] = n

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 validator.py out_file tc_file")
    else:
        validate(sys.argv[1], sys.argv[2])
