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
        int blur;
        int nfa;
        int num_lines;
        double total_length;
        double total_width;
        bool use_clahe;
        double clipLimit;
        int tileGrid;
    };

    // 宣言時だけstatic
    static double get_line_width(const cv::Mat& gray, const cv::Point2f& p1, const cv::Point2f& p2);
    static std::vector<LineInfo> detect_LSD(const cv::Mat& original, int blur_size, int nfa_thresh,
                                            bool use_clahe = true, double clipLimit = 2.0, int tileGrid = 8);
    static Result find_best(const cv::Mat& original,
                            bool use_clahe = true, int tileGrid = 13);
    static void draw_lines(const cv::Mat& original, const std::vector<LineInfo>& lines, int blur_size,
                            bool use_clahe, double clipLimit, int tileGrid);
    static cv::Mat homography(const cv::Mat& input_gray, float shrink_ratio = 0.05f);
    static std::tuple<Result, std::vector<LineInfo>, cv::Mat> run_detection(const cv::Mat& original);
    static inline cv::Mat apply_clahe(const cv::Mat& gray, double clipLimit = 2.0, int tileGrid = 8);
};

#endif // MISORA2_CRACKS_SIZE_HPP
