//============================================================================
// Name        : testing.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
using namespace std;

int main() {
	// Create an empty property tree object
	using boost::property_tree::ptree;
    ptree pt;

	string jsonS = "{\"key\":\"value\",\"otherKey\":\"otherValue\"}";
	stringstream lineStream;

	lineStream << jsonS;

	cout << "reading json" << endl;
	read_json(lineStream, pt);
	cout << "reading age" << endl;
	string result = pt.get<string>("otherKey");

	cout << "age: " << result << endl;



	return 0;
}
