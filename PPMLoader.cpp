/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/



#include "PPMLoader.hpp"

#include "Image.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <locale>
#include <limits>
#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cstring>
#include <sstream>
#include "macros.hpp"

using namespace rw::sensor;
using namespace rw::loaders;

namespace {

std::string quote(const std::string& str)
{
    const std::string q = "'";
    return q + str + q;
}

template <class X>
inline
std::pair<bool, X> toX(const std::string& str)
{
    using namespace std;
    pair<bool, X> nothing(false, X());

    if( str == "nan" || str == "NaN" )
        return make_pair(true,std::numeric_limits<X>::quiet_NaN() ); //
    if( str == "-inf" || str == "inf")
        return make_pair(true,std::numeric_limits<X>::infinity() ); //
    istringstream buf(str);
    X x;
    buf >> x;
    if (!buf) return nothing;

    string rest;
    buf >> rest;
    if (buf) return nothing;
    else return make_pair(true, x);
}

std::pair<bool, unsigned long> toULong(const std::string& str)
{
    return toX<unsigned long>(str);
}


std::pair<bool, unsigned int> toUInt(const std::string& str)
{
    return toX<unsigned int>(str);
}


	// We have this utility here so that we can be sure that IO errors don't
    // take place. It is nasty for IO errors to not be discovered. Also we can
    // speed up things by reading the input stream into a buffer, if we want to,
    // without having to change the user of Reader.
    class Reader
    {
    public:
      Reader(const std::string& filename) : _filename(filename)
		{
			_in = new std::ifstream(_filename.c_str(), std::ifstream::binary);
			if (!(in().is_open())){
				RW_THROW("Can't open file " << _filename);
			}
			okOrThrow();
		}

		~Reader() {
			in().close();
			delete _in;
		}

		void read(char* out, size_t size)
		{
			in().read(out, size);
			okOrThrow();
		}

		void getline(std::string &out)
		{
			std::getline(in(), out);
			okOrThrow();
		}

		void ignore(size_t cnt)
		{
			in().ignore(cnt);
			okOrThrow();
		}

		int get()
		{
			int tmp = -1;
			tmp = in().get();
			okOrThrow();
			return tmp;
		}

		template <class X>
		X getNextAsciiString() {
			X output = 0;
			char tmp = in().get();
			std::string input = "";

			//Skip spaces before the value
			while(std::isspace(tmp, std::locale())!=0) {
				tmp = in().get();
			}

			//Get the string
			do{
				input.push_back(tmp);
				tmp = in().get();
			}while(std::isspace(tmp, std::locale())==0);

			//Convert the string to a value
			std::pair<bool, unsigned long> uLongPair= toULong(input);
			if(uLongPair.first) {
				output = (X)uLongPair.second;
			}
			else {
				RW_THROW("Can't convert ascii color value \"" << input<<"\" to ungisned long");
			}

			return output;
		}


		const std::string &fileName() const { return _filename; }

	private:
		bool ok() { return !in().fail(); }
		void okOrThrow() { if (!ok()) RW_THROW("IO error."); }

		std::ifstream& in() { return *_in; }
		std::ifstream* _in;
		const std::string _filename;

    };

	enum fileType {none, binary, ascii, binary_pgm, ascii_pgm};

	void getHeader(Reader& streamIn, fileType &type, unsigned int &width, unsigned int &heigth, unsigned long &pixelMax) {
		//Get Header
		char tmp;
		std::string input;
		std::string comment = "No comment";
		unsigned int step=0;

		//Store the old locale
		std::string locale = setlocale(LC_ALL, NULL); 
		//Sets up the locale to standard english.
		setlocale(LC_ALL, "C");


		while(step<4) {
			tmp = streamIn.get();

			//Comment
			if(tmp=='#') {
				streamIn.getline(comment);
				continue;
			}

			//Space
			if(std::isspace(tmp, std::locale())) {
				continue;
			}

			input.clear();
			do{
				input.push_back(tmp);
				tmp = streamIn.get();
			}while(std::isspace(tmp, std::locale())==0);

			std::pair<bool, unsigned int> uIntPair;
			std::pair<bool, unsigned long> uLongPair;
			switch(step) {
			case 0:
				//Magic number
				if(input.compare("P6")==0) {
					type = binary;
				} else if(input.compare("P5")==0) {
					type = binary_pgm;
				} else if(input.compare("P2")==0) {
					type = ascii_pgm;
				} else if(input.compare("P3")==0) {
					type = ascii;
				}
				else {
					type = none;
					RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Can't find the magic code");
				}
				step = 1;
				break;
			case 1:
				//Width
				uIntPair = toUInt(input);
				if(uIntPair.first) {
					width = uIntPair.second;
				}
				else {
					width = 0;
					RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Can't find image width \"" << input<<"\"");
				}

				step = 2;
				break;
			case 2:
				//Heigth
				uIntPair = toUInt(input);
				if(uIntPair.first) {
					heigth = uIntPair.second;
				}
				else {
					heigth = 0;
					RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Can't find image heigth \"" << input<<"\"");
				}
				step = 3;
				break;
			case 3:
				//Pixel max value
				uLongPair = toULong(input);
				if(uLongPair.first) {
					pixelMax = uLongPair.second;
				}
				else {
					pixelMax = 0;
					RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Can't find max pixel value \"" << input<<"\"");
				}
				step = 100; //Header done;
				break;
			default:
				RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Error in the loader");
				break;
			}
		}
		//Restore the locate setting
		setlocale(LC_ALL, locale.c_str());   
	}

	void getBinaryImageData(Reader& streamIn, rw::sensor::Image* img) {
		//Pars data from the file to the image
		streamIn.read(img->getImageData(),img->getDataSize());
	}

	template <class X>
	void getAsciiImageData(Reader& streamIn, rw::sensor::Image* img) {
		//Verify the expected size of the image with the value type
		size_t totalSize = img->getWidth()*img->getHeight()*img->getNrOfChannels()*sizeof(X);
		if(totalSize != img->getDataSize()) {
			RW_THROW("Can't open file " << quote(streamIn.fileName()) << " - Error in the getAsciiImageData");
		}

		//Pars data from the file to the image
		X* imgData = (X*) img->getImageData();
		for(size_t i = 0; i<(img->getWidth()*img->getHeight()*img->getNrOfChannels()); i+=img->getNrOfChannels()) {
			for(unsigned int c=0; c<img->getNrOfChannels(); ++c) {
				imgData[i+c] = streamIn.getNextAsciiString<X>();
			}
		}
	}
}

rw::sensor::Image* PPMLoader::loadImage(const std::string& filename){
    return PPMLoader::load(filename);
}

rw::sensor::Image* PPMLoader::load(const std::string& filename)
{
	Reader readerObj(filename);

	fileType type = none;
	unsigned int width = 0;
	unsigned int heigth = 0;
	unsigned long pixelMax = 0;

	//Get Header
	getHeader(readerObj, type, width, heigth, pixelMax);

	//Get color depth
	Image::PixelDepth depth = Image::Depth8U;
	if(pixelMax < 256) {
		depth = Image::Depth8U;
	}
	else if(pixelMax < 65536) {
		depth = Image::Depth16U;
	}
	else if(pixelMax < 4294967296) {
		depth = Image::Depth32S;
	}
	else {
		RW_THROW("Can't open file " << quote(readerObj.fileName()) << " - Max pixel value too large");
	}

	//Get Image data
	rw::sensor::Image* img = NULL;
	if(type==binary_pgm || type==ascii_pgm){
	    img = new rw::sensor::Image(width, heigth, Image::GRAY, depth);
	} else {
	    img = new rw::sensor::Image(width, heigth, Image::RGB, depth);
	}

	if(type==binary || type==binary_pgm) {
		getBinaryImageData(readerObj, img);
	}
	else if(type==ascii || type==ascii_pgm) {
		if(depth==Image::Depth8U)
			getAsciiImageData<uint8_t>(readerObj, img);
		else if(depth==Image::Depth16U)
			getAsciiImageData<uint16_t>(readerObj, img);
		else if(depth==Image::Depth32S)
			getAsciiImageData<int32_t>(readerObj, img);
	}
	else {
		RW_THROW("Can't open file " << quote(readerObj.fileName()) << " - Unknown type");
	}

    return img;
}
