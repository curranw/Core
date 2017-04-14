

#include "FileIn.h"
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>

using std::string;
using std::istringstream;
using std::vector;
using std::ifstream;
using std::cin;
using std::stod;

namespace cio {
void mkdir_p(string pathname) {
    char *path = (char*)pathname.c_str();
    char *token = NULL;
    char *context = NULL;
    char delims[] = "/";

#ifdef _WIN32
    token = strtok_s(path, delims, &context);
#else
    token = strtok_r(path, delims, &context);
#endif

    string tmp = "";

    // keep going until there are no tokens left
    while (token != NULL) {
        // append the next token to the path
        tmp += token;

        // Create the directory
#ifdef _WIN32
        _mkdir(tmp.c_str());
#else
        mkdir(tmp.c_str(), 666);
#endif

        tmp += "/";  // append a / to the path for the next token and get it

#ifdef _WIN32
        token = strtok_s(NULL, delims, &context);
#else
        token = strtok_r(NULL, delims, &context);
#endif
    }
//    delete[]token;
//    delete[]path;
//    delete[]context;
//    delete[]&delims;
}

void stripFilePath(string *file_name) {
    istringstream iss(*file_name);
    string token;
    // Returns the last element in the filepath
    while (getline(iss, token, '/')) {}
    *file_name = token;
}

void stripExtension(string *file_name) {
    size_t found;
    found = file_name->find('.');
    file_name->resize(found);
}

//! Divides a string by 'separator'
vector<string> divide(string my_string, const string &separator) {
    vector<string> divided;
    while (my_string.find(separator) != string::npos) {
        // Find the separator
        size_t found = my_string.find(separator);
        // Cut at separator
        divided.push_back(my_string.substr(0, found));
        // Remove to separator
        my_string.erase(my_string.begin(), my_string.begin() + found + 1);
    }
    divided.push_back(my_string);
    return divided;
}

bool getYesNo(string question) {
    while (true) {
        string user_entry;
        printf("%s (y/n)\t", question.c_str());
        cin >> user_entry;
        if (user_entry == "y") {
            return true;
        } else if (user_entry == "n") {
            return false;
        } else {
            printf("Unrecognized entry. ");
        }
    }
}

string detectSeparator(string file_name) {
    vector<string> divided = divide(file_name, ".");
    if (divided.back() == "csv") {
        return ",";
    } else if (divided.back() == "xls") {
        return "\t";
    } else {
        unrecognizedExtension(divided.back());
        return "";
    }
}

bool fileExists(string filename) {
    ifstream myfile(filename);
    bool exists = myfile.good();
    myfile.close();
    return exists;
}

//! Reads variables (double) from a file to access by name (string)
VarMap readVariableFile(string file_name) {
    vector2<string> raw_data = read2<string>(file_name);
    VarMap  processed_data;
    for (vector<string> &r : raw_data) {
        auto p = make_pair(r[0], stod(r[1].c_str()));
        processed_data.insert(p);
    }
    return processed_data;
}


void variableNotFound(string var_name) {
    printf("Variable %s not found!\n", var_name.c_str());
    system("pause");
}

//! Error that prints when a file was not opened.
void failedFile(string file_name) {
    printf("Failed to open %s.\n", file_name.c_str());
}

//! Error that prints, pauses, and closes the program.
void unrecognizedExtension(string extension_name) {
    printf("Unknown extension %s: ", extension_name.c_str());
    printf("please specify separator type. Aborting.\n");
    system("pause");
}

//! Error that prints, pauses, and closes the program when pair detection fails.
void notPair(string file_name) {
    printf("Error! %s ", file_name.c_str());
    printf("does not contain pair values. Aborting.\n");
    system("pause");
}
}
