#!/bin/bash

# The first line in any Shell script must be the shebang (#!) followed by the path to the interpreter.
# In our case thats #!/bin/bash

FOLDER_NAME="/tmp/ros"     # assign the string /tmp/ros to the variable FOLDER_NAME.
                            # Attention variable declaration in BASH allowes no " " (space) between the variable name and the value!

echo "Creating folder $FOLDER_NAME"     # Printing the message 'Creating folder' followed by the variable. Note the "$" infornt of the variable name.
mkdir -p "$FOLDER_NAME"     # Creating a directory using the variable with argument -p (Create intermediate folders if not existent)
echo "Return value of previous command: $?" # All commands return a value. 0 means everything was ok.
cd "$FOLDER_NAME"           # Change directory inside the new folder
WORKDIR=$(pwd)              # Assign the output of the command pwd (print working directory) to the variable WORKDIR
echo "Current folder: $WORKDIR"           # Print the current working directory. Here we use "(...)" to create a subshell. Subshells can be used to directly process output of commands
cd .. && echo "Current folder: $(pwd)"  # Move one directory up (cd ..) and execute (if first command [cd ..] succeeds) the same as above
cd "$FOLDER_NAME"           # Change directory again
exit 0                      # Return a value of 0 (everything ok) to the calling process. This is not necessary as the script will return the value of the last command executed.