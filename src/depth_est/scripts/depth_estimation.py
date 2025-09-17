#!/usr/bin/env python3
import rospy
import os
import numpy as np
import PIL.Image as pil
import matplotlib as mpl
import matplotlib.cm as cm
import time
import torch
from torchvision import transforms
import networks
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthModel():
    def __init__(self, weights_folder, model, use_cuda):
        self.weights_folder = weights_folder
        self.model_type = model
        self.use_cuda = use_cuda
        self.load_model()
    def load_model(self):
        if torch.cuda.is_available() and self.use_cuda:
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        print("-> Loading model from ", self.weights_folder)
        encoder_path = os.path.join(self.weights_folder, "encoder.pth")
        decoder_path = os.path.join(self.weights_folder, "depth.pth")

        encoder_dict = torch.load(encoder_path)
        decoder_dict = torch.load(decoder_path)

        # extract the height and width of image that this model was trained with
        self.feed_height = encoder_dict['height']
        self.feed_width = encoder_dict['width']

        print("   Loading pretrained encoder")
        self.encoder = networks.LiteMono(model="lite-mono" if self.model_type == "base" else f"lite-mono-{self.model_type}",
                                        height=self.feed_height,
                                        width=self.feed_width)

        model_dict = self.encoder.state_dict()
        self.encoder.load_state_dict({k: v for k, v in encoder_dict.items() if k in model_dict})

        self.encoder.to(self.device)
        self.encoder.eval()

        print("   Loading pretrained decoder")
        self.depth_decoder = networks.DepthDecoder(self.encoder.num_ch_enc, scales=range(3))
        depth_model_dict = self.depth_decoder.state_dict()
        self.depth_decoder.load_state_dict({k: v for k, v in decoder_dict.items() if k in depth_model_dict})

        self.depth_decoder.to(self.device)
        self.depth_decoder.eval()


        
def predict(data, model, depth_publisher):
    image = data.data
    bridge = CvBridge()
    with torch.no_grad():
        start_time = time.time()  # Thời gian bắt đầu toàn bộ predict

        # Image preprocessing
        preprocess_start = time.time()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        input_image = pil.fromarray(rgb_image)
        original_width, original_height = input_image.size
        input_image = input_image.resize((model.feed_width, model.feed_height), pil.LANCZOS)
        input_image = transforms.ToTensor()(input_image).unsqueeze(0)
        preprocess_time = time.time() - preprocess_start

        # predict
        inference_start = time.time()
        input_image = input_image.to(model.device)
        features = model.encoder(input_image)
        outputs = model.depth_decoder(features)
        disp = outputs[("disp", 0)]
        inference_time = time.time() - inference_start

        # postprocess
        postprocess_start = time.time()
        disp_resized = torch.nn.functional.interpolate(
            disp, (original_height, original_width), mode="bilinear", align_corners=False)
        disp_resized_np = disp_resized.squeeze().cpu().numpy()

        vmax = np.percentile(disp_resized_np, 99)  # Use 95th percentile for normalization
        vmin = disp_resized_np.min()
        normalized_depth = (disp_resized_np - vmin) / (vmax - vmin + 1e-8)  # Avoid division by zero
        normalized_depth = 1 - normalized_depth  # Invert depth for visualization
        
        # Lấy kích thước của normalized_depth
        height, width = normalized_depth.shape

        # Tính số hàng/cột cho padding 15%
        top_bottom_rows = int(height * 0.15)  # 15% chiều cao cho trên và dưới
        left_right_cols = int(width * 0.15)   # 15% chiều rộng cho trái và phải

        # # Đặt các vùng biên thành 0 (màu đen)
        # normalized_depth[:top_bottom_rows, :] = 0.0  # Padding trên
        normalized_depth[-top_bottom_rows:, :] = 0.0  # Padding dưới
        # normalized_depth[:, :left_right_cols] = 0.0  # Padding trái
        # normalized_depth[:, -left_right_cols:] = 0.0  # Padding phải

        # normalized_depth[(normalized_depth >= 0.75) & (normalized_depth <= 1.0)] = 0.0
        depth_mono16 = (normalized_depth * 65535).astype(np.uint16)
        # Convert to ROS Image message with mono16 encoding
        output_depth_img = bridge.cv2_to_imgmsg(depth_mono16, encoding='mono16')
        output_depth_img.header = data.header
        postprocess_time = time.time() - postprocess_start

        # Publish
        depth_publisher.publish(output_depth_img)

        total_time = time.time() - start_time
        # print(f"Preprocess time: {preprocess_time:.4f}s | Inference time: {inference_time:.4f}s | Postprocess time: {postprocess_time:.4f}s | Total: {total_time:.4f}s")
        # print("Proceed!")    
def main():
    rospy.init_node("depth_estimation", anonymous=True)

    model_type = rospy.get_param("~model", "base")   
    weights_folder = rospy.get_param("~weights_folder", "/workspace/catkin_ws_ov/src/depth_est/checkpoints/") + model_type
    use_cuda = rospy.get_param("~cuda", True)

    model = DepthModel(weights_folder, model_type, use_cuda)
    depth_publisher = rospy.Publisher("/camera/depth_image", Image, queue_size=10)
    rospy.Subscriber("/camera/image_raw", Image, lambda x: predict(x, model, depth_publisher))
    print("Using device", model.device)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node")

if __name__ == '__main__':
    main()