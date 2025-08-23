#ifndef MISORA2_CRACKS_SIZE_HPP
#define MISORA2_CRACKS_SIZE_HPP

#include <opencv2/opencv.hpp>
#include "misora2_cracks/lsd.h"
#include <vector>
#include <tuple>

class CracksSize {
public:
    struct LineInfo {
        cv::Point2f p1, p2;
        double length; // mm
        double width;  // mm
    };

    struct Result {
        int threshold;
        int blur;
        int nfa;
        int num_lines;
        double total_length;
        double total_width;
    };

    // 宣言時だけstatic
    static double get_line_width(const cv::Mat& bin_blur, const cv::Point2f& p1, const cv::Point2f& p2);
    static std::vector<LineInfo> detect_LSD(const cv::Mat& original, int threshold_val, int blur_size, int nfa_thresh);
    static Result find_best(const cv::Mat& original);
    static void draw_lines(const cv::Mat& original, const std::vector<LineInfo>& lines, int threshold_val, int blur_size);
    static cv::Mat homography(const cv::Mat& input_gray, float shrink_ratio = 0.05f);
    static std::tuple<Result, std::vector<LineInfo>, cv::Mat> run_detection(const cv::Mat& original);
};

#endif // MISORA2_CRACKS_SIZE_HPP
