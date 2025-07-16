// detection.cpp
#include "misora2_cracks/detection.hpp"

const std::vector<std::vector<int>> Detection::skeleton = {
    {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7},
    {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}
};

const std::vector<cv::Scalar> Detection::posePalette = {
    cv::Scalar(255, 128, 0), cv::Scalar(255, 153, 51), cv::Scalar(255, 178, 102), cv::Scalar(230, 230, 0),
    cv::Scalar(255, 153, 255), cv::Scalar(153, 204, 255), cv::Scalar(255, 102, 255), cv::Scalar(255, 51, 255),
    cv::Scalar(102, 178, 255), cv::Scalar(51, 153, 255), cv::Scalar(255, 153, 153), cv::Scalar(255, 102, 102),
    cv::Scalar(255, 51, 51), cv::Scalar(153, 255, 153), cv::Scalar(102, 255, 102), cv::Scalar(51, 255, 51),
    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255)
};

const std::vector<int> Detection::limbColorIndices = {9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16};
const std::vector<int> Detection::kptColorIndices = {16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9};

cv::Scalar Detection::generateRandomColor(int numChannels) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);
    cv::Scalar color;
    for (int i = 0; i < numChannels; i++) {
        color[i] = dis(gen);
    }
    return color;
}

std::vector<cv::Scalar> Detection::generateRandomColors(int class_names_num, int numChannels) {
    std::vector<cv::Scalar> colors;
    for (int i = 0; i < class_names_num; i++) {
        colors.push_back(generateRandomColor(numChannels));
    }
    return colors;
}

void Detection::plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
    std::unordered_map<int, std::string>& names)
{
    cv::Mat mask = img.clone();
    for (size_t i = 0; i < result.size(); i++)
    {
        float left, top;
        left = result[i].bbox.x;
        top = result[i].bbox.y;
        // int color_num = i;
        int& class_idx = result[i].class_idx;
        rectangle(img, result[i].bbox, color[result[i].class_idx], 2);

        // try to get string value corresponding to given class_idx
        std::string class_name;
        auto it = names.find(class_idx);
        if (it != names.end()) {
            class_name = it->second;
        }
        else {
            std::cerr << "Warning: class_idx not found in names for class_idx = " << class_idx << std::endl;
            // then convert it to string anyway
            class_name = std::to_string(class_idx);
        }

        if (result[i].mask.rows && result[i].mask.cols > 0)
        {
            mask(result[i].bbox).setTo(color[result[i].class_idx], result[i].mask);
        }
        std::stringstream labelStream;
        labelStream << class_name << " " << std::fixed << std::setprecision(2) << result[i].conf;
        std::string label = labelStream.str();

    	cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
        cv::Rect rect_to_fill(left - 1, top - text_size.height - 5, text_size.width + 2, text_size.height + 5);
        cv::Scalar text_color = cv::Scalar(255.0, 255.0, 255.0);
        rectangle(img, rect_to_fill, color[result[i].class_idx], -1);

        putText(img, label, cv::Point(left - 1.5, top - 2.5), cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
    }
    addWeighted(img, 0.6, mask, 0.4, 0, img); //add mask to src
    resize(img, img, img.size());
}

std::tuple<cv::Mat, cv::Mat> Detection::plot_results(
    cv::Mat img,
    std::vector<YoloResults>& results,
    std::vector<cv::Scalar> color,
    std::unordered_map<int, std::string>& names)
{
    cv::Mat image_with_box = img.clone();
    cv::Mat extracted_patch;  // 射影変換後の画像（1つだけ処理対象）

    for (const auto& res : results) {
        // 枠描画
        cv::rectangle(image_with_box, res.bbox, color[res.class_idx], 2);

        // クラス名取得
        std::string class_name = names.count(res.class_idx) ? names[res.class_idx] : std::to_string(res.class_idx);

        // ラベル描画
        std::stringstream labelStream;
        labelStream << class_name << " " << std::fixed << std::setprecision(2) << res.conf;
        std::string label = labelStream.str();

        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
        int left = res.bbox.x;
        int top = res.bbox.y;

        cv::Rect label_rect(left - 1, top - text_size.height - 5, text_size.width + 2, text_size.height + 5);
        cv::rectangle(image_with_box, label_rect, color[res.class_idx], -1);
        cv::putText(image_with_box, label, cv::Point(left - 1.5, top - 2.5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // マスクがある場合のみ処理
        if (res.mask.rows > 0 && res.mask.cols > 0) {
            cv::Mat cropped_img = img(res.bbox).clone();

            // マスクを255スケールに変換
            cv::Mat mask_bin;
            res.mask.convertTo(mask_bin, CV_8UC1, 255);

            // 輪郭検出
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask_bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (contours.empty()) continue;

            // 最大輪郭選択
            size_t max_idx = 0;
            double max_area = 0.0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }

            std::vector<cv::Point> contour = contours[max_idx];

            // 四隅推定
            cv::Point tl, tr, br, bl;
            double min_sum = 1e9, max_sum = -1e9, min_diff = 1e9, max_diff = -1e9;
            for (const auto& pt : contour) {
                int sum = pt.x + pt.y;
                int diff = pt.x - pt.y;
                if (sum < min_sum) { min_sum = sum; tl = pt; }
                if (sum > max_sum) { max_sum = sum; br = pt; }
                if (diff < min_diff) { min_diff = diff; bl = pt; }
                if (diff > max_diff) { max_diff = diff; tr = pt; }
            }

            std::vector<cv::Point2f> quad_pts = {tl, bl, br, tr};

            // 縮小処理
            float shrink_ratio = 0.05f;
            cv::Point2f center(0, 0);
            for (const auto& pt : quad_pts) center += pt;
            center *= 1.0f / quad_pts.size();

            std::vector<cv::Point2f> shrunk_quad;
            for (const auto& pt : quad_pts) {
                cv::Point2f dir = center - pt;
                shrunk_quad.push_back(pt + dir * shrink_ratio);
            }

            // 射影変換先サイズ決定（縦横比保持）
            float widthA = cv::norm(quad_pts[0] - quad_pts[3]);
            float widthB = cv::norm(quad_pts[1] - quad_pts[2]);
            float width = (widthA + widthB) / 2.0f;

            float heightA = cv::norm(quad_pts[0] - quad_pts[1]);
            float heightB = cv::norm(quad_pts[3] - quad_pts[2]);
            float height = (heightA + heightB) / 2.0f;

            int warped_width = std::round(width);
            int warped_height = std::round(height);

            std::vector<cv::Point2f> dst_pts = {
                {0, 0},
                {0, float(warped_height - 1)},
                {float(warped_width - 1), float(warped_height - 1)},
                {float(warped_width - 1), 0}
            };

            // 射影変換
            cv::Mat M = cv::getPerspectiveTransform(shrunk_quad, dst_pts);
            cv::warpPerspective(cropped_img, extracted_patch, M, cv::Size(warped_width, warped_height));
            break;  // 複数物体を処理したい場合は break を削除
        }
    }

    return {extracted_patch, image_with_box};
}
