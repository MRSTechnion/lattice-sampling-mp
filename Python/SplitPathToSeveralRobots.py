if __name__ == "__main__":
    fpath = '/home/itai/CLionProjects/OmplLattices/cmake-build-release/testpath.path'
    with open('robot1.path', 'w') as f1, open('robot2.path', 'w') as f2:
        with open(fpath, 'r') as file:
            # Read each line one by one
            for line in file:
                elements = [float(e) for e in line.strip().split()]
                if len(elements) == 0:
                    continue
                f1.write("{} {} {}\n".format(elements[0], elements[1], elements[2]))
                f2.write("{} {} {}\n".format(elements[3], elements[4], elements[5]))
