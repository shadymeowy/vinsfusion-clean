#include "netvladdb.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>

int main()
{
	try {
		// Path to your ONNX model
		std::string model_path = "/datasets/netvlad_db.onnx";

		// Create NetVLADDB instance with history size of 1024 descriptors
		NetVLADDB db(model_path, 1024);

		// Create a dummy HWC CV_8UC3 image of size 1024x640
		cv::Mat img1(1024, 640, CV_8UC3);
		std::mt19937 rng(42);
		std::uniform_int_distribution<int> u8(0, 255);
		for (int y = 0; y < img1.rows; ++y) {
			for (int x = 0; x < img1.cols; ++x) {
				cv::Vec3b &px = img1.at<cv::Vec3b>(y, x);
				px[0] = static_cast<uint8_t>(u8(rng)); // B
				px[1] = static_cast<uint8_t>(u8(rng)); // G
				px[2] = static_cast<uint8_t>(u8(rng)); // R
			}
		}

		// Query the database with img1
		std::vector<int> best_idx;
		std::vector<float> best_values;
		db.query(img1, best_idx, best_values);

		std::cout << "Query 1 top-k indices: ";
		for (int i = 0; i < (int)best_idx.size(); ++i)
			std::cout
				<< best_idx[i]
				<< (i + 1 < (int)best_idx.size() ? ", " : "\n");

		std::cout << "Query 1 top-k values: ";
		for (int i = 0; i < (int)best_values.size(); ++i)
			std::cout << best_values[i]
				  << (i + 1 < (int)best_values.size() ? ", " :
									"\n");

		for (int i = 0; i < 2; ++i) {
			db.query(img1, best_idx, best_values);
		}

		// Make a slightly different image (img2) by adding small random deltas
		cv::Mat img2 = img1.clone();
		std::uniform_int_distribution<int> delta(0, 10);
		for (int y = 0; y < img2.rows; ++y) {
			for (int x = 0; x < img2.cols; ++x) {
				cv::Vec3b &px = img2.at<cv::Vec3b>(y, x);
				for (int c = 0; c < 3; ++c) {
					int val = static_cast<int>(px[c]) +
						  delta(rng);
					px[c] = static_cast<uint8_t>(
						std::min(val, 255));
				}
			}
		}

		// Query the database with img2
		db.query(img2, best_idx, best_values);

		std::cout << "Query 2 top-k indices: ";
		for (int i = 0; i < (int)best_idx.size(); ++i)
			std::cout
				<< best_idx[i]
				<< (i + 1 < (int)best_idx.size() ? ", " : "\n");

		std::cout << "Query 2 top-k values: ";
		for (int i = 0; i < (int)best_values.size(); ++i)
			std::cout << best_values[i]
				  << (i + 1 < (int)best_values.size() ? ", " :
									"\n");

		// Benchmarking
		auto start = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < 100; ++i) {
			db.query(img1, best_idx, best_values, false);
		}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		std::cout << "Average query time over 100 runs: "
			  << (elapsed.count() / 100.0) << " seconds\n";

	} catch (const Ort::Exception &e) {
		std::cerr << "ONNX Runtime error: " << e.what() << "\n";
		return 1;
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 2;
	}
	return 0;
}
