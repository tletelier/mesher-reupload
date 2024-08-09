import os

# NUM = 5
NAME = 'guide'
# "bothFiducials"
# "guide"
# "Fiducials1"

for i in [0,1,2,3,4,5,6,7]:
    NUM = i
    print(i)
    filename = f"{NAME}{NUM}.m3d"
    folder_path = "data"

    # Construct the full file path
    file_path = os.path.join(folder_path, filename)
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()

            # Extract the numbers based on the known positions
            nodes_number = lines[1]
            elements_number = lines[int(nodes_number) + 5]

            # Print the extracted numbers
            print("Number of Nodes:", nodes_number)
            print("Number of Elements:", elements_number)

    except FileNotFoundError:
        print(f"File '{file_path}' not found.")
    except IndexError:
        print("Could not extract numbers. Invalid file structure.")
    except ValueError:
        print("Could not convert a line to an integer.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    print('--------------')

