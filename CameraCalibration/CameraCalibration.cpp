// CameraCalibration.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;



int main(int argc, char* argv[])
{
	int n_boards = 0;//コマンドライン引数より決定
	float image_sf = 0.5f;
	float delay = 1.f;
	int board_w = 0;
	int board_h = 0;

	//適切でないコマンドライン引数の時終了
	if (argc <4 || 6 <= argc)
	{
		cout << "\nERROR: Wrong number of input parameters\n";
		cv::waitKey(0);
		return -1;
	}
	board_w = atoi(argv[1]);
	board_h = atoi(argv[2]);
	n_boards = atoi(argv[3]);
	if (argc > 4) delay =  atof(argv[4]);
	if (argc > 5) image_sf = atof(argv[5]);

	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);

	cv::VideoCapture capture(2);//DroidCamがIndex:2
	if (!capture.isOpened())
	{
		cout << "\nCouldn't open the camera\n";
		cv::waitKey(0);
		return -1;
	}

	//格納場所を確保
	vector<vector<cv::Point2f>> image_points;
	vector<vector<cv::Point3f>> object_points;

	//コーナーの画像を取り込む．取り込みに成功した（ボード上のすべてのコーナーが見つかった）画像が
	//n_board枚集まるまでループする
	double last_captured_timestamp = 0;
	cv::Size image_size;

	while (image_points.size()<(size_t)n_boards)
	{
		cv::Mat image0, image;
		capture >> image0;
		image_size = image0.size();
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);

		//ボードを探す
		vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_sz, corners);

		//描画する
		cv::drawChessboardCorners(image, board_sz, corners, found);

		//良いボードが見つかれば，データに加える
		double timestamp = (double)clock()/CLOCKS_PER_SEC;

		if (found && timestamp-last_captured_timestamp>1)
		{
			last_captured_timestamp = timestamp;
			image ^= cv::Scalar::all(255);

			cv::Mat mcorners(corners);		//データをコピーしない
			mcorners *= (1. / image_sf);	//コーナーの座標をスケーリングする
			image_points.push_back(corners);
			object_points.push_back(vector<cv::Point3f>());
			vector<cv::Point3f>& opts = object_points.back();
			opts.resize(board_n);
			for (int i = 0; i < board_n; i++)
			{
				opts[i] = cv::Point3f((float)(i / board_w), (float)(i % board_w), 0.f);
			}
			cout << "Collected our" << (int)image_points.size() << "of" << n_boards << "needed chess board images\n" << endl;
		}
		cv::imshow("Calibration", image);

		if ((cv::waitKey(30)&255)==27) return -1;
	}
	//画像の読み取りループ終了

	cv::destroyWindow("Calibration");
	cout << "\n\n***CALIBRATING THE CAMERA***\n" << endl;

	//カメラのキャリブレーションを行う
	cv::Mat intrinsic_matrix, distortion_coeffs;
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
	);

	//内部パラメータと歪み係数を保存する
	cout << "***DONE!\n\nReprojection error is " << err << "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);

	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients" << distortion_coeffs;

	fs.release();


	//これらの行列を読み込み直す
	fs.open("intrinsics.xml", cv::FileStorage::READ);
	cout << "\nimage width" << (int)fs["image_width"];
	cout << "\nimage height" << (int)fs["image_height"];

	cv::Mat intrinsic_matrix_roaded, distortion_coeffs_roaded;
	fs["camera_matrix"] >> intrinsic_matrix_roaded;
	fs["distortion_coefficients"] >> distortion_coeffs_roaded;
	cout << "\nintrinsic matrix:" << intrinsic_matrix_roaded;
	cout << "\ndistortion coefficients:" << distortion_coeffs_roaded << endl;

	//後続フレーム全てに対して用いる補正用のマップを作成する
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix_roaded,
		distortion_coeffs_roaded,
		cv::Mat(),
		intrinsic_matrix_roaded,
		image_size,
		CV_16SC2,
		map1,
		map2
	);

	//カメラ画像を画面に表示する
	//元画像と歪み補正後の画像を表示
	//
	for (;;)
	{
		cv::Mat image, image0;
		capture >> image0;
		if (image0.empty()) break;
		cv::remap(
			image0,
			image,
			map1,
			map2,
			cv::INTER_LINEAR,
			cv::BORDER_CONSTANT,
			cv::Scalar()
		);
		cv::imshow("Undistorted", image);
		if((cv::waitKey(30) & 255)==27)break;
	}

	return 0;
}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
