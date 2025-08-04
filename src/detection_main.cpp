#include "misora2_cracks/detection.hpp"

int main(int argc, char *argv[]) {
    std::string image_file;

    if(argc > 1)image_file = argv[1];
    else {
        std::cout << "Please input image path" << std::endl;
        return 0;
    }
    // 初期設定-----------------------------------------------------
    if (!std::filesystem::exists(Detection::MODEL_PATH)) {
        std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
        throw std::runtime_error("Model file not found.");
    }
    // モデルの準備
    AutoBackendOnnx model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER);
    // std::cout << image_file << 1 << std::endl;
    std::vector<cv::Scalar> colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    std::unordered_map<int, std::string> names = model.getNames();
    // 画像の読み込み
    cv::Mat img = cv::imread(image_file, cv::IMREAD_UNCHANGED);
    // 推論実行
    std::vector<YoloResults> objs = model.predict_once(
        img,
        Detection::CONF_THRESHOLD,
        Detection::IOU_THRESHOLD,
        Detection::MASK_THRESHOLD,
        Detection::CONVERSION_CODE
    );
    // -------------------------------------------------------------------------------------
    if (img.empty()) {
        std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
        return -1;
    }

    std::cout << "Processing image: " << image_file << " (size: " << img.size() << ")" << std::endl;
    // 結果の描画
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    std::cout << "Num of objects :" << size(objs) << std::endl;
    auto [trimmed, boxed] = Detection::plot_results(img, objs, colors, names);
    if(trimmed.channels() == 1) std::cout << "Not found" << std::endl;
    else 
    {
        std::cout << "trimmed: " << trimmed.size() << std::endl;
        cv::imshow("with box",boxed);
        cv::imshow("trimmed",trimmed);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}
