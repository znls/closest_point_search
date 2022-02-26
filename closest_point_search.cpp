/*
参考にしたサイト

*/

#include<vector>
#include<string>
#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
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

// 線形補間
double lerp(double t, double n1, double n2, double t1, double t2) {
    return n1 + ((n2 - n1) / (t2 - t1)) * (t - t1);
}

// 三次元座標の位置を線形補間
eph3d eph3d_lerp(double t, eph3d A, eph3d B) {
    eph3d lerp_pos;
    lerp_pos.epoch = t;
    lerp_pos.x = lerp(t, A.x, B.x, A.epoch, B.epoch);
    lerp_pos.y = lerp(t, A.y, B.y, A.epoch, B.epoch);
    lerp_pos.z = lerp(t, A.z, B.z, A.epoch, B.epoch);

    return lerp_pos;

}

// 2点間の距離
double dist(eph3d pos1, vec3d pos2) {
    return sqrt(pow((pos1.x - pos2.x),2) + pow((pos1.y - pos2.y),2) + pow((pos1.z - pos2.z),2));
}

// distanceの導関数
double dist_prime(eph3d pos1, vec3d pos2) {
    return 1 / (2 * dist(pos1, pos2));
}

eph3d closest_point_search(const vector<eph3d>& eph, const vec3d& obs)
{
    /*
    最接近地点(X)に一番近いポイントのひとつ前のポイント(A)とひとつ後のポイント(B)を求める
    */
    eph3d A;
    eph3d B;
    vector<double> vec_normOA;
    // 軌道暦と観測地点(O)との間の距離をすべて計算する
    for (auto& e : eph) {
        double normOA = dist(e, obs);
        vec_normOA.push_back(normOA);
    }
    // 計算した距離の最小値のインデックスを取得
    vector<double>::iterator min_iter = min_element(vec_normOA.begin(), vec_normOA.end());
    size_t min_index = distance(vec_normOA.begin(), min_iter);

    // A, Bの間に最接近地点が存在する
    A = eph[min_index - 1];
    B = eph[min_index + 1];

    // ニュートン・ラフソン法で使う変数
    double t0 = A.epoch;
    eph3d temp_pos = A;
    double ti;
    double eps = 1; // エポックtの更新量

    // ニュートン・ラフソン法
    while (eps < 1E-8) {
        // t_i+1を求める
        ti = t0 - dist(temp_pos, obs) / dist_prime(temp_pos, obs);
        // 値の更新
        eps = ti - t0;
        t0 = ti;
        // 更新したtで線形補間
        temp_pos = eph3d_lerp(t0, A, B);
    }

    return temp_pos;
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

    // 軌道暦読み込み
    ifstream ifs_eph("eph.csv");
    vector<eph3d> eph;
    while (getline(ifs_eph, line)) {
        vector<string> strvec = split(line, ',');
        eph3d e;
        e.epoch = stod(strvec[0]);
        e.x = stod(strvec[1]);
        e.y = stod(strvec[2]);
        e.z = stod(strvec[3]);
        eph.push_back(e);
    }

    eph3d closest_point = closest_point_search(eph, obs_point);

    // 結果出力
    cout << "最接近地点:" << endl;
    cout << "epoch = " << fixed << setprecision(10) <<closest_point.epoch << endl;
    cout << "x = " << fixed << setprecision(3) << closest_point.x << endl;
    cout << "y = " << fixed << setprecision(3) << closest_point.y << endl;
    cout << "z = " << fixed << setprecision(3) << closest_point.z << endl;

}