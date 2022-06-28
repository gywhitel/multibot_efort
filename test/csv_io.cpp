#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <eigen3/Eigen/Core>

using namespace std;
#define I 0
#define O 0

typedef Eigen::Matrix<double,1,18> joint_angle_tribot;
typedef std::vector<joint_angle_tribot> joint_trajectory;

/*
* @brief
* @param filename A csv file
* @return a n x 18 matrix of joint angles  
*/
joint_trajectory readCSV(std::string filename)
{
    ifstream in(filename, ios::in);
    string line, element;
    joint_trajectory joint_traj;
    stringstream sstream;
    while (getline(in, line))
    {
        // DEBUG wrong output
        sstream<<line;
        cout<<sstream.str();
        joint_angle_tribot joint_position;
        while (getline(sstream, element, ','))
        {
            try
            {
                // cout<<element;
                joint_position<<stod(element);
            }
            // catch(const std::exception& e)
            catch(const std::invalid_argument& e)
            {
                std::cerr << e.what();
                cout<<" Invalid argument:"<<element<<"\n";
                continue;
            }
        }
        joint_traj.push_back(joint_position);
        sstream.clear();
    }
    return joint_traj;
}

int main(int argc, char** argv)
{
    joint_trajectory vector = readCSV(argv[1]);
    // for (int i = 0; i < vector.size(); i++)
    // {
    //     for (int j = 0; j < vector[i].cols(); j++)
    //     {
    //         cout<<vector[i](j)<<',';
    //     }
    //     cout<<endl;
    // }
#if O
    ofstream out;
    out.open("data.csv", ios::out);
    out<<"A"<<','<<"B"<<','<<"C"<<','<<"D"<<endl;
    for (int i = 0; i < 10; i++)
    {
        out<<i<<','<<2*i<<','<<3*i<<','<<4*i<<endl;
    }
    out.close();
#endif

#if I
    // ifstream in(argv[1], ios::in);
    // string line, element;
    // vector<vector<int>> data;
    // stringstream sstream;
    // bool header = true;
    // while (getline(in, line))
    // {
    //     if (!header)
    //     {
    //         sstream<<line;
    //         vector<int> data_line;
    //         while (getline(sstream, element, ','))
    //         {
    //             data_line.push_back(stoi(element));
    //         }
    //     data.push_back(data_line);
    //     sstream.clear();
    //     }
    //     header = false;
    // }

    ifstream in(argv[1], ios::in);
    string line, element;
    vector<vector<int>> data;
    stringstream sstream;
    while (getline(in, line))
    {
        sstream<<line;
        vector<int> data_line;
        while (getline(sstream, element, ','))
        {
            try
            {
                data_line.push_back(stoi(element));
            }
            // catch(const std::exception& e)
            catch(const std::invalid_argument& e)
            {
                std::cerr << e.what() << '\n';
                cout<<"Table head\n";
                continue;
            }
        }
        data.push_back(data_line);
        sstream.clear();
    }


    for (int i = 0; i < data.size(); i++)
    {
        for (auto iter = data[i].begin(); iter != data[i].end(); iter++)
            cout<<*iter<<" ";
        cout<<endl;
    }
    
#endif
}