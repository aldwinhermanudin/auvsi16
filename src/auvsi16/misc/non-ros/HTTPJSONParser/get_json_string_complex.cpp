#include "include/rapidjson/reader.h"
#include <iostream>
#include <vector>

using namespace rapidjson;
using namespace std;

struct JSONHandler {
  vector<string> json_element;
  void clearJSONVector(){this->json_element.clear();}
  bool Null() {return true; }
  bool Bool(bool b) {
    if(b)this->json_element.push_back("true");
    else this->json_element.push_back("false");
    return true; }
  bool Int(int i) {return true; }
  bool Uint(unsigned u) {return true; }
  bool Int64(int64_t i) {return true; }
  bool Uint64(uint64_t u) {return true; }
  bool Double(double d) {return true; }
  bool RawNumber(const char* str, SizeType length, bool copy) {return true;}
  bool String(const char* str, SizeType length, bool copy) {
    string buffer_string;
    buffer_string.append(str);
    this->json_element.push_back(buffer_string);
    return true;
  }
  bool StartObject() {return true; }
  bool Key(const char* str, SizeType length, bool copy) {
    string buffer_string;
    buffer_string.append(str);
    this->json_element.push_back(buffer_string);
    return true;
  }
  bool EndObject(SizeType memberCount) {return true; }
  bool StartArray() {return true; }
  bool EndArray(SizeType elementCount) {return true; }
};

int main() {
  //const char* json = "{\"dockingBaySequence\":[{\"symbol\":\"cruciform\",\"color\":\"red\"},{\"symbol\":\"cruciform\",\"color\":\"red\"}]}";
  const char* json = "asdasdsad";
  JSONHandler handler;
  Reader reader;
  StringStream ss(json);
  handler.clearJSONVector();
  reader.Parse(ss, handler);
  if (reader.HasParseError()) {
              cout << "error" << endl;
                return EXIT_FAILURE;
  }

  StringStream ss2(json);
  handler.clearJSONVector();
  reader.Parse(ss2, handler);

  vector<string>::iterator iter;
  for( iter = handler.json_element.begin(); iter != handler.json_element.end(); iter++ ) {
    cout << *iter << endl;
  }

  return 0;
}
