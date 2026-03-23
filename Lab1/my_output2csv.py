import sys

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 my_output2csv.py <input> <output>")
        return
    
    with open(sys.argv[1], 'r') as f_in, open(sys.argv[2], 'w') as f_out:
        f_out.write("id,routing_result\n")
        for i, line in enumerate(f_in):
            f_out.write(f"{i},{line.strip().replace(',', '')}\n")

if __name__ == '__main__':
    main()
