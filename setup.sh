#!/usr/bin/env bash

if [ "$(uname)" != "Darwin" ]; then
    echo "This script only runs on Mac!"
    exit 1
fi

command -v brew >/dev/null || { echo "Homebrew not installed.  Installing..."; /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"; }

command -v ipopt >/dev/null || { echo "Ipopt not installed.  Installing..."; brew install dartsim/dart/ipopt; }

command -v cmake >/dev/null || { echo "CMake not installed.  Installing..."; brew install cmake; }

#check if libglew exists
echo "int main(){}" | gcc -o /dev/null -x c - -lglew 2>/dev/null
if [ $? -eq 1 ]; then
	echo "LibGlew not installed.  Installing..."
	brew install glew
fi

command -v gnuplot >/dev/null || { echo "Gnuplot not installed.  Installing..."; brew install gnuplot --with-qt; }

echo ""
echo ""
echo "DONE!"