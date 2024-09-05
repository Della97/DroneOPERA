#!/bin/bash

# Set variables
NS3_URL="https://www.nsnam.org/releases/ns-allinone-3.40.tar.bz2"
NETSIMULYZER_REPO_URL="https://github.com/usnistgov/NetSimulyzer-ns3-module.git"
DEST_DIR="includes/ns3"
NS3_ARCHIVE="ns-allinone-3.40.tar.bz2"
NS3_DIR="ns-allinone-3.40"

# Download NS-3
curl -O $NS3_URL

# Check if the download was successful
if [ $? -ne 0 ]; then
  echo "Failed to download NS-3."
  exit 1
fi

# Extract the NS-3 archive
tar xjf $NS3_ARCHIVE -C /tmp

# Check if the extraction was successful
if [ $? -ne 0 ]; then
  echo "Failed to extract NS-3 archive."
  exit 1
fi

# Create the destination directory if it doesn't exist
mkdir -p $DEST_DIR

# Move the extracted NS-3 directory to the destination directory
mv /tmp/$NS3_DIR $DEST_DIR

# Navigate to the NS-3 directory
cd $DEST_DIR/$NS3_DIR/ns-3.40

# Clone the NetSimulyzer module into the contrib folder
git clone $NETSIMULYZER_REPO_URL contrib/netsimulyzer

# Move to netsim to patch
cd contrib/netsimulyzer

# Patch
git fetch
git checkout v1.0.10

# Check if the clone was successful
if [ $? -ne 0 ]; then
  echo "Failed to clone NetSimulyzer module."
  exit 1
fi

# Restore position
cd ../../../../..

# Clone rapidjson
git clone https://github.com/Tencent/rapidjson.git

# Restore pos
cd ns3/ns-allinone-3.40/ns-3.40

# Configure the NS-3 project with examples, tests, and MPI enabled
./ns3 configure --enable-examples --enable-tests --enable-mpi

# Build the NS-3 project
./ns3 build

# Check if the build was successful
if [ $? -ne 0 ]; then
  echo "Failed to build NS-3."
  exit 1
fi

echo "NS-3 and NetSimulyzer module have been successfully downloaded, configured, and built."

# Clean up the temporary directory and downloaded archive (optional)
rm -rf /tmp/$NS3_DIR
rm $NS3_ARCHIVE
