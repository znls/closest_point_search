/*
参考にしたサイト
http://www.sousakuba.com/Programming/gs_near_pos_on_line.html
*/

#include<vector>
#include<string>
#include<fstream>
#include<sstream>
#include<iostream>
#include<math.h>
#include<algorithm>

using namespace std;


struct vec3d {
double x, y, z;
};

struct eph3d {
double epoch, x, y, z;
};

// csvファイルのパース用関数
vector<string> split(string& input, char delimiter) {
    istringstream stream(input);
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }

    return result;

}

vec3d closest_point_search(const vector<eph3d>& eph, const vec3d& obs)
{
    /*
    最接近地点(X)に一番近いポイントのひとつ前のポイント(A)とひとつ後のポイント(B)を求める
    */
    eph3d A;
    eph3d B;
    vector<double> vec_normOA;
    // 軌道暦と観測地点(O)との間の距離をすべて計算する
    for (auto& e : eph) {
        double normOA = sqrt(pow((e.x - obs.x),2) + pow((e.y - obs.y),2)
                               + pow((e.z - obs.z),2));
        vec_normOA.push_back(normOA);
    }
    // 計算した距離の最小値のインデックスを取得
    vector<double>::iterator min_iter = min_element(vec_normOA.begin(), vec_normOA.end());
    size_t min_index = distance(vec_normOA.begin(), min_iter);

    // A, Bを決定
    A = eph[min_index - 1];
    B = eph[min_index + 1];

    /*
    ABの単位ベクトルを求める
    */
    vec3d AB;  // ベクトルAB
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    AB.z = B.z - A.z;
    // ベクトルABの長さ
    double length_AB = sqrt( (AB.x*AB.x)+(AB.y*AB.y)+(AB.z*AB.z) );
    vec3d unitAB;  // ABの単位ベクトル
    unitAB.x = AB.x / length_AB;
    unitAB.y = AB.y / length_AB;
    unitAB.z = AB.z / length_AB;

    /*
    最接近地点をX，観測地点をOとして，AXの距離を求める
        AXの距離 = ABの単位ベクトル ・ ベクトルAO
            ※ABの単位ベクトルとベクトルAOの内積
    */
    vec3d AO;
    AO.x = obs.x - A.x;
    AO.y = obs.y - A.y;
    AO.z = obs.z - A.z;

    double distanceAX = unitAB.x*AO.x + unitAB.y*AO.y + unitAB.z*AO.z;

    /*
    最接近地点Xを求める
        X = A + (ABの単位ベクトル*AXの距離)
    */

   vec3d X;
   X.x = A.x + (unitAB.x * distanceAX);
   X.y = A.y + (unitAB.y * distanceAX);
   X.z = A.z + (unitAB.z * distanceAX);

   return X;
}

int main() {

    string line;

    // 観測地点読み込み
    ifstream ifs_obs("obs_point.csv");
    vec3d obs_point;
    getline(ifs_obs, line);
    vector<string> strvec = split(line, ',');
    obs_point.x = stod(strvec[0]);
    obs_point.y = stod(strvec[1]);
    obs_point.z = stod(strvec[2]);

    // デバッグライト
    // cout << "観測地点:" << endl;
    // cout << "x = " << obs_point.x << endl;
    // cout << "y = " << obs_point.y << endl;
    // cout << "z = " << obs_point.z << endl;

    // 軌道暦読み込み
    ifstream ifs_eph("eph.csv");
    vector<eph3d> eph;
    int counter = 0;
    while (getline(ifs_eph, line)) {
        vector<string> strvec = split(line, ',');
        eph3d e;
        e.epoch = stod(strvec[0]);
        e.x = stod(strvec[1]);
        e.y = stod(strvec[2]);
        e.z = stod(strvec[3]);
        eph.push_back(e);
        counter++;
    }

    // デバッグライト
    // cout << "軌道暦" << endl;
    // for (auto& e : eph) {
    //     cout << "epoch = " << e.epoch << endl;
    //     cout << "x = " << e.x << endl;
    //     cout << "y = " << e.y << endl;
    //     cout << "z = " << e.z << endl;
    // }

    vec3d closest_point = closest_point_search(eph, obs_point);

    // 結果出力
    cout << "最接近地点:" << endl;
    cout << "x = " << closest_point.x << endl;
    cout << "y = " << closest_point.y << endl;
    cout << "z = " << closest_point.z << endl;

}