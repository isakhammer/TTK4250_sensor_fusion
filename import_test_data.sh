
unzip $( pwd )/slam_project/slam_handout.zip
cp $( pwd )/slam_handout/tests/test_data.pickle $( pwd )/slam_project/tests/
rm -Rf $( pwd )/slam_handout
FILE=$( pwd )/slam_project/tests/

if [ -f "$FILE" ]; then
    echo "$FILE exists."
else
    echo "$FILE does not exist."
fi


