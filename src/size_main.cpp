#include "misora2_cracks/size.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Please input <image_path>" << std::endl;
        return -1;
    }

    cv::Mat original = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (original.empty()) {
        std::cerr << "Image not found: " << argv[1] << std::endl;
        return -1;
    }
    auto start = std::chrono::high_resolution_clock::now();
    
    auto [best, lines, corrected] = CracksSize::run_detection(original);
    if (best.num_lines == 0 || lines.empty()) {
        std::cerr << "Best result is empty or no detected lines.\n";
        return 0;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "detect5 [処理時間] " << duration.count() << " ms\n";
    CracksSize::draw_lines(corrected, lines, best.blur);

    std::cout << "Blur: " << best.blur
              << ", NFA: " << best.nfa << '\n';
    std::cout << "Lines: " << best.num_lines << '\n'
              << "Total Length (mm): " << best.total_length << '\n'
              << "Total width (mm): " << best.total_width << '\n';

    return 0;
}