START_DIR=$(pwd)

command -v brew >/dev/null || { echo "Homebrew not installed.  Installing..."; /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"; }

command -v ipopt >/dev/null || { echo "Ipopt not installed.  Installing..."; brew install dartsim/dart/ipopt; }

command -v cmake >/dev/null || { echo "CMake not installed.  Installing..."; brew install cmake; }

#check if libglew exists
echo "int main(){}" | gcc -o /dev/null -x c - -lglew 2>/dev/null
if [ $? -eq 1 ]; then
	echo "LibGlew not installed.  Installing..."
	brew install glew
fi

if [ ! -d "range-v3" ]; then
	echo "Installing Range-v3..."
	rangeurl=$(curl -s https://api.github.com/repos/ericniebler/range-v3/releases/latest | grep tarball_url | cut -d '"' -f 4)
	curl -L -o range.tar.gz $rangeurl
	mkdir range-v3
	tar -xzf range.tar.gz -C range-v3 --strip-components=1
	rangedir="$(pwd)/range-v3/include"
	rm range.tar.gz
fi

echo "int main(){}" | gcc -o /dev/null -x c - -lgtest 2>/dev/null
if [ $? -eq 1 ]; then
	echo "GTest and GMock not installed.  Installing..."
	GTEST_DIR="gtest-1.8.0"
	curl -L -o gtest-1.8.0.tar.gz "https://github.com/google/googletest/archive/release-1.8.0.tar.gz"
	mkdir $GTEST_DIR
	tar -xzf gtest-1.8.0.tar.gz -C $GTEST_DIR --strip-components=1

	mkdir -p $GTEST_DIR/mybuild
	cd $GTEST_DIR/mybuild
	cmake -G"Unix Makefiles" ..
	make
	sudo cp -r ../googletest/include/gtest /usr/local/include
	sudo cp -r ../googlemock/include/gmock /usr/local/include
	sudo cp googlemock/lib*.a /usr/local/lib
	sudo cp googlemock/gtest/lib*.a /usr/local/lib
	cd $START_DIR
	rm -r $GTEST_DIR
	rm gtest-1.8.0.tar.gz
fi

command -v gnuplot >/dev/null || { echo "Gnuplot not installed.  Installing..."; brew install gnuplot --with-qt; }

echo ""
echo ""
echo "DONE!"
if [ ! -z ${rangedir+x} ]; then
	echo "Range3_INCLUDE_DIR for LocalProperties.cmake: $rangedir"
fi