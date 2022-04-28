
exit_cmds = ['exit', 'x', 'no']
forward_cmds = ['f', 'forward']
backward_cmds = ['b', 'backward']

def main():
    random_arr = [a**2 for a in range(10)]
    tot_len = len(random_arr)
    idx = 0
    
    while(True):
        # get input 
        input_cmd = input("do something: ").lower()            

        # do stuff
        if input_cmd in exit_cmds:
            break
        else:
            if input_cmd.isnumeric():
                input_cmd = int(input_cmd)
                if input_cmd < tot_len and input_cmd >= 0:
                    idx = input_cmd
                else:
                    print("Not a valid index.")
            else:
                if input_cmd in forward_cmds:
                    if idx == tot_len-1:
                        print("You reached the end of the array")
                    else:
                        idx += 1
                elif input_cmd in backward_cmds:
                    if idx == 0:
                        print("You reached the beginning of the array")
                    else:
                        idx -= 1
            print(random_arr[idx])
        

if __name__ == "__main__":
    main()