import subprocess

DIR = "/home/taiga/Desktop/U/mesher/"
NUM = "5"
NAME = "guide"
VIEWER = True

# "bothFiducials"
# "guide"
# "Fiducials1"

# Compilar
cmake_process = subprocess.Popen(["cmake", "../src/"], cwd=DIR + "build")
cmake_process.wait()  # Wait for cmake to finish

if cmake_process.returncode == 0:
    make_process = subprocess.Popen(["make"], cwd=DIR + "build", shell=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    universal_newlines=True)
    make_output, make_error = make_process.communicate()

    if make_process.returncode == 0:
        print(make_output)
        make_output = make_output.strip()

        if make_output.endswith("[100%] Built target mesher_roi"):
            mesher_roi_process = subprocess.Popen(["../build/mesher_roi"], cwd=DIR + "data")
            mesher_roi_process.wait()

            # Check if mesher_roi executed successfully

            if mesher_roi_process.returncode == 0:
                test_file_process = subprocess.Popen(["../build/mesher_roi", "-d", f"{NAME}.mdl", "-s", NUM, "-u", f"{NAME}{NUM}", "-m", "-v"],
                                                    cwd=DIR + "data")
                test_file_process.wait()

                # Check if test_file executed successfully
                # ../build/mesher_roi -d guide.mdl -s 4 -u guide4 -m -v
                if test_file_process.returncode == 0 and VIEWER:
                    print(VIEWER)
                    process = subprocess.Popen(["../../view/build/viewer", "-m", f"{NAME}{NUM}.m3d"], cwd=DIR + "data",
                                             bufsize=0, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE)

                    process.stdin.write(('%s\n' % "n").encode())

                    output, error = process.communicate()
                    print(output)
                else:
                    print(f"Error running test_file: {test_file_process.returncode}")
            else:
                print(f"Error running mesher_roi: {mesher_roi_process.returncode}")
        else:
            print(make_error)
            print("Error during make. Make output:")
            print(make_output)
            print("Make error:")
    else:
        print(f"Error during make: {make_process.returncode}")
else:
    print(f"Error during cmake: {cmake_process.returncode}")