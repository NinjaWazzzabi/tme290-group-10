# Combine all messages into a temporary file
touch messages/tmp
cat messages/custom-messages.odvd >   messages/tmp
cat messages/opendlv-messages.odvd >> messages/tmp

# Generate the headerfile
cluon-msc -cpp --out=messages/messages.hpp messages/tmp

# Remove temporary file
rm messages/tmp

# Distribute the new message header to all projects
cp messages/messages.hpp action-planner/src/
cp messages/messages.hpp aimpoint-driver/src/
cp messages/messages.hpp cone-detection/src/
cp messages/messages.hpp kiwi-detection/src/
cp messages/messages.hpp debug-drawer/src/

# Remove dangling header file
rm messages/messages.hpp