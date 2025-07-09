import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Float64MultiArray

MIN_PIXEL=1500
MAX_CONTOUR_AREA = 20000
CONTOURS_MIN_AREA = 500 
class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        #이미지 설정
        # self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.sub_image_type='compressed'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        # self.sub_image_original = self.create_subscription(CompressedImage, '/camera/image/compressed', self.cbFindLane,1)
        self.sub_image_original = self.create_subscription(CompressedImage, '/camera/image/compressed', self.cbFindLane,1)
        self.pub_image_lane = self.create_publisher(CompressedImage, '/detect/image/compressed', 1)

        #디버깅용
        # self.debug_mask_pub = self.create_publisher(CompressedImage, '/detect/debug_mask/compressed', 1)
        self.debug_contour_pub = self.create_publisher(CompressedImage, '/debug/contours/compressed', 1)


        #버드뷰 버리고 라인 너무 가까움
            
        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1) #여기서 라인 중심값

        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

        self.cvBridge = CvBridge()

        #여기 초기화한 변수 들이 무엇일까
        self.counter = 1 # #3프레임 중 1개만 처리하기 위한 카운트


        self.lane_fit_bef = np.array([0.0, 0.0, 0.0])
        self.reliability_left_line=0
        self.reliability_right_line=0
        # __init__에 추가


        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))


    def cbFindLane(self, image_msg):
        if self.counter % 3 != 0: #3프레임 중 1개만 처리 (10fps 기준이면 실제 3.3fps)
            self.counter += 1
            return
        else:
            self.counter = 1
        make_lane_flag=False
        self.left_fitx = None
        self.right_fitx = None
        sliding_flag=False

        right_count = 0
            
        """
        self.sub_image_type = 'compressed'  
        self.pub_image_type = 'compressed'
        """
        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # BGR → Grayscale 변환
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 밝은 영역(흰색 계열) 추출
        _, mask = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
        # 작은 반사 흔적 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
       
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours = [cnt for cnt in contours if cv2.contourArea(cnt) > CONTOURS_MIN_AREA]

        # for i, cnt in enumerate(contours):
        #     area = cv2.contourArea(cnt)
        #     print(f"Contour {i}: {len(cnt)} points, area: {area:.2f}")

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 필터링 후, 면적 기준 내림차순 정렬 (큰 순서대로)
        #area는 500이상 20000미만
        # contours = sorted(
        #     [cnt for cnt in contours
        #     if CONTOURS_MIN_AREA < cv2.contourArea(cnt) < MAX_CONTOUR_AREA],
        #     key=cv2.contourArea,
        #     reverse=True
        # )   
        #area 20000이상인거 버릴거

        ####추가
        contours = sorted(
            [cnt for cnt in contours if CONTOURS_MIN_AREA < cv2.contourArea(cnt)],
            key=cv2.contourArea,
            reverse=True
        )

        if len(contours) == 0:
            return  # 아무 것도 없으면 탈출

        cnt = contours[0]
        area = cv2.contourArea(cnt)

        # 너무 큰 경우 → 가로선으로 연결되어 하나로 붙었을 가능성
        if area >= 20000:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / w if w != 0 else float('inf')
            print(f"[컨투어 비율] height={h}, width={w}, aspect_ratio={aspect_ratio:.2f}")

            mid_x = x + w // 2
            mask[:, mid_x - 10:mid_x + 10] = 0  # 가운데 세로선 삽입
            print(f"[분할] 중앙 x={mid_x} 에서 컨투어 분할 시도")

            # 분할 후 다시 컨투어 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 다시 필터링 및 정렬 (작은 것부터 정렬하면 top2 뽑기 불편)
            contours = sorted(
                [cnt for cnt in contours if CONTOURS_MIN_AREA < cv2.contourArea(cnt) < MAX_CONTOUR_AREA],
                key=cv2.contourArea,
                reverse=True
            )
            print(f"[분할 후] contours 개수: {len(contours)}")

        ###추가끝
       
        # 디버깅 출력
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            print(f"Contour {i}: {len(cnt)} points, area: {area:.2f}")
        # 디버그용 컨투어 시각화 이미지 (마스크를 컬러로 바꾸고 컨투어 그리기)
        
        debug_contour_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug_contour_img, contours, -1, (0, 255, 255), 2)  # 노란색 윤곽선
        self.debug_contour_pub.publish(
        self.cvBridge.cv2_to_compressed_imgmsg(debug_contour_img, 'jpg'))

        left_mask = np.zeros_like(mask)
        right_mask = np.zeros_like(mask)

        left_count = 0
        if not contours:
            #차선검출 x
            return 
        print(f"contours개수 :{len(contours)}")
        if len(contours) == 1:
            #세로차선제외로직

            #기울기추출
            #/이거면 왼쪽 차선 \이거면 오른쪽 차선
            # 컨투어에서 좌표 배열 가져오기
            cnt = contours[0]
            mask_single = np.zeros_like(mask)
            cv2.drawContours(mask_single, [cnt], -1, 255, -1)
            ys, xs = np.nonzero(mask_single)

            if len(xs) == 0:
                self.get_logger().warn("라인하나에서 픽셀검출실패")
                return
            poly = np.polyfit(xs, ys, 2)
            x_center = np.mean(xs)
            slope = 2 * poly[0] * x_center + poly[1]
            angle_rad = np.arctan(slope)
            angle_deg = np.degrees(angle_rad)

            height, width = mask.shape  
            mid_x = width // 2
            cx = int(x_center)

            # 각도 정규화: OpenCV는 -90~0도 사이로 리턴
            # 예: 우하향 "/" 형태는 angle ≈ -45
            #     좌하향 "\" 형태는 angle ≈ -135 또는 45 등

            if angle_deg > 10:
                print(f"좌회전 중 → 오른쪽 차선으로 판단 (angle = {angle_deg:.2f})")
                right_mask = mask_single
                right_count = len(xs)
                print(f"right_count : {right_count}")
                left_mask = np.zeros_like(mask)
                left_count = 0   
                sliding_flag=True   

            elif angle_deg < -10:
                print(f"우회전 중 → 왼쪽 차선으로 판단 (angle = {angle_deg:.2f})")
                left_mask = mask_single
                left_count = len(xs)
                print(f"len_count : {left_count}")
                right_mask = np.zeros_like(mask)
                right_count = 0
                sliding_flag=True  

            else:
                if cx < mid_x:
                    print(f"직선: 왼쪽 차선으로 판단 (cx = {cx})")
                    left_mask = mask_single
                    left_count = len(xs)
                    right_mask = np.zeros_like(mask)
                    right_count = 0
                else:
                    print(f"직선: 오른쪽 차선으로 판단 (cx = {cx})")
                    right_mask = mask_single
                    right_count = len(xs)
                    left_mask = np.zeros_like(mask)
                    left_count = 0
        if len(contours) >= 2:
            # 컨투어들을 면적 기준으로 정렬
        
            top2 = contours[:2]

            # 각 컨투어의 중심 x좌표 구하기
            #이거 기울기생각
            centers = []
            for cnt in top2:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    centers.append((cx, cnt))

            if len(centers) < 2:
                self.get_logger().warn("라인은 두개이지만 중심좌표계산 실패")
                return  # 둘 다 정상적인 중심을 못 구했으면 생략

            # 중심 x좌표 기준으로 왼쪽/오른쪽 나누기
            height, width = mask.shape
            mid_x = width // 2

            # x 좌표 기준으로 정렬 (왼쪽 먼저)
            centers = sorted(centers, key=lambda x: x[0])

            # 왼쪽 컨투어 → 왼쪽 차선
            left_cx, left_cnt = centers[0]
            left_mask = np.zeros_like(mask)
            cv2.drawContours(left_mask, [left_cnt], -1, 255, -1)
            left_count = np.count_nonzero(left_mask)

            # 오른쪽 컨투어 → 오른쪽 차선
            right_cx, right_cnt = centers[1]
            right_mask = np.zeros_like(mask)
            cv2.drawContours(right_mask, [right_cnt], -1, 255, -1)
            right_count = np.count_nonzero(right_mask)

      
        try:
            if sliding_flag:
                raise Exception("Sliding flag activated")
            
            if left_count > MIN_PIXEL//2:
                self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, left_mask)
                self.mov_avg_left = np.append(self.mov_avg_left, [self.left_fit], axis=0)
                make_lane_flag=True

            if right_count > MIN_PIXEL//2:
                self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, right_mask)
                self.mov_avg_right = np.append(self.mov_avg_right, [self.right_fit], axis=0)
                make_lane_flag=True

        except Exception as e:
            self.get_logger().warn(f"[fit_from_lines 실패] fallback to sliding_window: {e}")

            if left_count > MIN_PIXEL//2:
                self.left_fitx, self.left_fit = self.sliding_windown(left_mask, 'left')
                self.mov_avg_left = np.array([self.left_fit])
                make_lane_flag=True
            else:
                self.get_logger().warn(f"슬라이딩 했지만 왼쪽 픽셀 수 부족 :{left_count}")


            if right_count > MIN_PIXEL//2:
                self.right_fitx, self.right_fit = self.sliding_windown(right_mask, 'right')
                self.mov_avg_right = np.array([self.right_fit])
                make_lane_flag=True
            else:
                self.get_logger().warn(f"슬라이딩 했지만 오른쪽 픽셀 수 부족 :{right_count}")

        MOV_AVG_LENGTH = 5 #최근 5개 프레임의 계수만 평균에 반영

        #곡선 차선의 이차함수 계수들 이동 평균으로 부드럽게 만드는 부분
        self.left_fit = np.array([
            np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])
        self.right_fit = np.array([
            np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])

        if self.mov_avg_left.shape[0] > 1000:#누적 배열이 1000개 이상 쌓이면, 오래된 것 날리고 최근 5개만 남김
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]
        if make_lane_flag:
            self.make_lane(cv_image, left_count, right_count)# 컨투어 마스크 기반 픽셀 수 전달
   
    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = (
            (nonzerox >
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) &
            (nonzerox <
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin))
                )

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        if len(x) < 10 or len(y) < 10:
            raise ValueError(f"[fit_from_lines] too few points: x={len(x)}, y={len(y)}")

        if np.all(x == x[0]):
            raise ValueError("[fit_from_lines] x값이 모두 같아서 polyfit 불가능")

        if np.any(np.isnan(x)) or np.any(np.isnan(y)):
            raise ValueError("[fit_from_lines] NaN 포함됨")


        lane_fit = np.polyfit(y, x, 2)

        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
        print(f" fit_from_lines: x={x.shape}, y={y.shape}")
        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):

        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        out_img = np.dstack((img_w, img_w, img_w)) * 255

        midpoint = np.int_(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 20

        window_height = np.int_(img_w.shape[0] / nwindows)

        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_current = lane_base

        margin = 50

        minpix = 50

        lane_inds = []

        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            cv2.rectangle(
                out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            good_lane_inds = (
                (nonzeroy >= win_y_low) &
                (nonzeroy < win_y_high) &
                (nonzerox >= win_x_low) &
                (nonzerox < win_x_high)
                ).nonzero()[0]

            lane_inds.append(good_lane_inds)

            if len(good_lane_inds) > minpix:
                x_current = np.int_(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]
        print(f"[sliding] window point count: {len(x)}")  # ① 픽셀 수 확인

        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
            print(f"[sliding] lane_fit: {lane_fit}")      # ② polyfit 결과 확인
        except Exception:
            lane_fit = self.lane_fit_bef
            print("[sliding] polyfit 실패 → 이전 계수 사용")

        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        if np.any(lane_fitx < 0) or np.any(lane_fitx > img_w.shape[1]):
            print("[경고] lane_fitx 중 이미지 폭을 벗어난 값 존재")

        return lane_fitx, lane_fit
    

    def make_lane(self, cv_image, left_fraction,  right_fraction):

        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        lane_state = UInt8()
        self.is_center_x_exist = False


        if left_fraction > MIN_PIXEL//2 and self.left_fitx is not None: # 노란선(좌측)이 충분히 검출됐을 때
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines,np.int_([pts_left]),isClosed=False,color=(0, 0, 255),thickness=25)
            #왼쪽 차선 (빨강색표시)

        if right_fraction > MIN_PIXEL//2  and self.right_fitx is not None: #흰선(우측)이 충분히 검출됐을 때
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines,np.int_([pts_right]),isClosed=False,color=(255, 255, 0),thickness=25)
            # 오른쪽 차선 (연노랑-연두 계열로 시각화)

        if right_fraction > MIN_PIXEL//2 and left_fraction > MIN_PIXEL//2 and self.right_fitx is not None and self.left_fitx is not None: #픽셀 수 충분할때
            centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
            self.is_center_x_exist=True

            pts = np.hstack((pts_left, pts_right))
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            lane_state.data = 2 #상태값 2 : 라인 흰색 노란색 둘다 있음

            cv2.polylines(color_warp_lines,np.int_([pts_center]),isClosed=False,color=(0, 255, 255), thickness=12)
            #중앙 추정선(노란색표시)

                # Draw the lane onto the warped blank image
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
            #차선 영역 전체 채우기(초록)

        elif right_fraction > MIN_PIXEL//2 and left_fraction <= MIN_PIXEL//2 and self.right_fitx is not None:
             #오른쪽 차선만 감지된 경우
            centerx = np.subtract(self.right_fitx, 140)
            self.is_center_x_exist=True

            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            lane_state.data = 3 #상태값 3: 흰색(오른쪽) 차선만 있음을 의미

            cv2.polylines(color_warp_lines,np.int_([pts_center]),isClosed=False,color=(0, 255, 255),thickness=12)
            #중앙선 추정선(노란색)

        elif right_fraction <= MIN_PIXEL//2 and left_fraction > MIN_PIXEL//2 and self.left_fitx is not None:
            #왼쪽차선만검출
            centerx = np.add(self.left_fitx, 140)
            self.is_center_x_exist=True
            # → 왼쪽 차선 기준으로 오른쪽으로 280 픽셀 이동해서 중심선 추정
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            lane_state.data = 1 #상태값 1 : 노란색 차선(왼쪽)만 있음

            cv2.polylines(color_warp_lines,np.int_([pts_center]),isClosed=False,color=(0, 255, 255),thickness=12)
            #중앙선추정(노란색)
        else:
            # print(f"right_fraction : {right_fraction} , left_fraction : {left_fraction}")
            # print(f"self.right_fitx : {self.right_fitx} , self.left_fitx : {self.left_fitx}")
            print(f"right_fraction : {right_fraction} , left_fraction : {left_fraction}")
            self.is_center_x_exist=False
            lane_state.data = 0 #상태값 0 : 라인 둘다 없음

        # self.pub_lane_state.publish(lane_state) #라인 상태 값 퍼블리시
        # self.get_logger().info(f'Lane state: {lane_state.data}')

        # Combine the result with the original image
        #cv2.addWeighted()는 OpenCV에서 두 이미지를 가중치(weight)를 두고 합성하는 함수야.
        #즉, 두 이미지를 "반투명하게 겹쳐서" 하나의 이미지처럼 보여주는 데 써.
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.is_center_x_exist: #중심값 존재하면 중심값 존재 = centerx존재
            # publishes lane center
            msg_desired_center = Float64()
            try:
                msg_desired_center.data = centerx.item(180)#centerx.item(350)는 y=350 라인에서 중심 x좌표 추출
            except (IndexError, ValueError) as e:
                index=len(centerx)//2
                msg_desired_center.data = centerx.item(index)
            
            #카메라프레임 밖 좌표찍으면 안보냄
            if msg_desired_center.data>=0 and msg_desired_center.data<=320:
                print(f"center_x : {msg_desired_center.data}")
                self.pub_lane.publish(msg_desired_center) #중앙차선 x좌표 발행
                self.pub_lane_state.publish(lane_state) #라인 상태 값 퍼블리시
                self.get_logger().info(f'Lane state: {lane_state.data}')
            else:
                print(f"갑자기 튄center_x : {msg_desired_center.data}")
                lane_state.data=0
                self.pub_lane_state.publish(lane_state) #라인 상태 값 퍼블리시
                self.get_logger().info(f'Lane state: {lane_state.data}')
        else:
            self.pub_lane_state.publish(lane_state) #라인 상태 값 퍼블리시
            self.get_logger().info(f'Lane state: {lane_state.data}')

        self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))

def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
