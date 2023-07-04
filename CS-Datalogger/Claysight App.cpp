//Copyright (c) 2023, John Simonis and The Ohio State University
//This code was written by John Simonis for the ClaySight research project at The Ohio State University.
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <ctime>
using namespace std;

#include <serialib.h> //Simple serial port library https://github.com/imabot2/serialib

#include <OpenXLSX.hpp> //Simple XLSX library https://github.com/troldal/OpenXLSX
using namespace OpenXLSX;

char SERIAL_PORT[12] = "\\\\.\\COM"; //This is a character array/string that represents our COM port
string Columns[6] = {"A", "B", "C", "D", "E"}; //Here we have all of the column letters represented as a string due to OpenXLSX requirements
string Delimiter = ","; //This is what seperates each of our columns in the rows of text (CSV formatting)
string InpFN; //This is declared for filename input
char Cell[10]; //Simple cell buffer

const string currentDateTime() { //This is a function that fetches the current date and time using the ctime library
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    localtime_s(&tstruct, &now);
    strftime(buf, sizeof(buf), "%Y-%m-%d @ %X", &tstruct);

    return buf;
}

int main()
{
    vector<string> Data; //This vector stores all of our raw CSV data
    serialib serial; //Serial port object
    XLDocument doc; //Workbook object


    for (int i = 1; i < 99; i++) { //This iterates through all serial ports until it finds the one with a baudrate of 57600 which is specific to ClaySight
        SERIAL_PORT[7] = to_string(i)[0];
        SERIAL_PORT[8] = to_string(i)[1];
        if (serial.openDevice(SERIAL_PORT, 57600) == 1) { //This baudrate is relatively nonstandard so this shouldnt interfere with other serial devices
            cout << "Claysight likely located on: " << SERIAL_PORT << endl;
            serial.closeDevice();
            break;
        }
    }

    char errorOpening = serial.openDevice(SERIAL_PORT, 57600); //Attempt to open the port on the first 57600 device

    if (errorOpening != 1) { 
        cout << "Error connecting to ClaySight. Close all other Serial COM apps!" << endl << "Press enter to exit."; 
        getchar(); 
        return 0; 
    }
    cout << "Successful connection to: " << SERIAL_PORT << endl << "Input XLSX File Name: "; //Outputs the proper com port.
    getline(cin, InpFN);
    try { doc.open(InpFN + ".XLSX"); } //Attempt to open the XLSX file if it already exists
    catch (exception) { doc.create(InpFN + ".XLSX"); } //Create it if it doesnt
    auto wks = doc.workbook().worksheet("Sheet1"); //Sets our scope to sheet1


    char buffer[20];

    cout << endl << "Written by John Simonis" << endl << "Copyright(C) 2023" << endl << "ClaySight (The Ohio State University)";

    this_thread::sleep_for(1.5s);//Normally pauses like this are bad practice, this is just to show the attributation.
 
    serial.writeString("SendData");//Sends the "SendData" string over serial to begin the debug process

    while(true) { //This will continue logging data to our vector until we reach a DataEND or the user closes the program.
        serial.readString(buffer, '\n', 100);
        cout << buffer;
        if (strstr(buffer,"DataEND")){
            break;
        }
        else {
            Data.push_back(buffer); //This is somewhat bad practice as the program could theoretically overrun with data, this works for ClaySight debug mode ONLY
        }
    }

    int counter = 0; //Here we store our counter and column counter in scope since we need to do a handful of operations
    int column = 0;
    size_t stringpos = 0;
    while (counter < Data.size()) {//This double while loop may look complicated but it is just parsing the string by its delimiter.
        while ((stringpos = Data[counter].find(Delimiter)) != string::npos) {//Also, it is attempting an int conversion on all strings so numerical data is represented properly.
            try { wks.cell(Columns[column] + to_string(counter + 1)).value() = int(stoi(Data[counter].substr(0, stringpos))); }
            catch(exception){ wks.cell(Columns[column] + to_string(counter + 1)).value() = Data[counter].substr(0, stringpos); }
                Data[counter].erase(0, stringpos + Delimiter.length());//Every time we remove a Delimited string we erase it from the original string
                column++;
        }
        try { wks.cell(Columns[column] + to_string(counter + 1)).value() = int(stoi(Data[counter])); } //This is outside the secondary while loop as it only triggers once there are no remaining Delimiters.
        catch(exception){ wks.cell(Columns[column] + to_string(counter + 1)).value() = Data[counter]; }
        column = 0;
        stringpos = 0;
        counter++;
    }

    wks.cell("G1").value() = "Last Run:"; //Here is where we use our date and time from the initial run of the program
    wks.cell("H1").value() = currentDateTime();
    
    doc.save();

    serial.closeDevice();

    cout << "Press enter to exit.";
    getchar(); //This is just an easy way to close the program through prompt, technically this is unecessary as the program would otherwise finish automatically.
    return 0;
}