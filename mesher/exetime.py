import subprocess

DIR = "/home/taiga/Desktop/U/mesher/"

# NUM = "7"
NAME = "bothFiducials"
# "bothFiducials"
# "guide"
# "Fiducials1"
IT = 1
for i in [0,1,2,3,4,5,6,7]:
    NUM = str(i)
    print(i)
    # Check if mesher_roi executed successfully

    sum_time = 0
    for i in range(IT):
        test_file_process = subprocess.Popen(["../build/mesher_roi", "-d", f"{NAME}.mdl", "-s", NUM, "-u", f"{NAME}{NUM}", "-m", "-v"],
                                            cwd=DIR + "data", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        test_file_process.wait()
        stdout, stderr = test_file_process.communicate()  # Capture stdout and stderr
        lista = stdout.decode('utf-8').split(' ')
        print(stdout.decode('utf-8').split(' '))
        if lista[0] == 'no':
            continue
        elapsed_time = int(stdout.decode('utf-8').split(' ')[5])
        print(elapsed_time)
        if test_file_process.returncode == 0:
            sum_time += elapsed_time
        else:
            print(f"Error running test_file: {test_file_process.returncode}")
    print(f"Elapsed time: {sum_time/IT} ms")
    print('-----')