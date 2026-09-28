#!/usr/bin/env python3
"""Publish finished host OCR jobs as source-stamped ROS messages and images."""
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tl_vision.msg import PlateText


def main():
    rospy.init_node('plate_result_bridge')
    directory = Path(rospy.get_param('~photo_dir'))/'ocr'
    font = ImageFont.truetype(rospy.get_param('~font','/root/ocr_font.ttf'),36)
    bridge = CvBridge()
    text_pub = rospy.Publisher('/inspection/plate_text',PlateText,queue_size=3,latch=True)
    report_pub = rospy.Publisher('/inspection/plate_report',String,queue_size=3,latch=True)
    image_pub = rospy.Publisher('/inspection/plate_image',Image,queue_size=1,latch=True)
    seen = set()
    while not rospy.is_shutdown():
        for path in sorted(directory.glob('*.result.json')):
            if path.name in seen:
                continue
            result = json.loads(path.read_text())
            job = json.loads((directory/(result['waypoint']+'.job.json')).read_text())
            selected = next((f for f in result['frames'] if f['valid'] and f['text']==result['text']),
                            result['frames'][0])
            capture = next(f for f in job['frames'] if f['source_stamp']==selected['source_stamp'])
            image = cv2.imread(capture['raw_image'])
            if image is None:
                raise RuntimeError('missing OCR source image')
            message = PlateText()
            stamp = selected['source_stamp']
            message.header.stamp = rospy.Time(stamp['secs'],stamp['nsecs'])
            message.header.frame_id = 'camera_optical_frame'
            message.index = 0
            message.x1,message.y1,message.x2,message.y2 = capture['box']
            message.raw = selected['raw']
            message.text = result['text']
            message.display = (message.text[:2]+'·'+message.text[2:]) if message.text else ''
            message.confidence = result['confidence']
            message.elapsed_ms = result['total_elapsed_ms']
            message.valid = result['valid']
            message.issue = ';'.join(result['issues'])
            color = (0,220,0) if message.valid else (0,165,255)
            x1,y1,x2,y2 = capture['box']
            cv2.rectangle(image,(x1,y1),(x2,y2),color,3)
            canvas = PILImage.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
            label = '车牌：%s  %.3f' % (message.display or '未识别',message.confidence)
            draw = ImageDraw.Draw(canvas)
            draw.text((x1,max(0,y1-52)),label,font=font,fill=(0,255,0) if message.valid else (255,165,0))
            annotated = cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR)
            target = directory/(result['waypoint']+'.annotated.png')
            cv2.imwrite(str(target),annotated)
            image_message = bridge.cv2_to_imgmsg(annotated,'bgr8')
            image_message.header = message.header
            text_pub.publish(message)
            image_pub.publish(image_message)
            payload = dict(result,source_stamp=stamp,annotated_image=str(target),
                           published_stamp=rospy.Time.now().to_sec())
            report_pub.publish(String(data=json.dumps(payload,ensure_ascii=False)))
            (directory/(result['waypoint']+'.published.json')).write_text(
                json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            line = '[车牌OCR] %s：%s；置信度%.3f；%d帧一致；图片：%s' % (
                result['waypoint'],message.display or '未识别',message.confidence,
                result['matching_frames'],target)
            with (directory/'ocr_terminal.txt').open('a',encoding='utf-8') as stream:
                stream.write(line+'\n')
            rospy.loginfo(line)
            seen.add(path.name)
        rospy.sleep(.05)


if __name__ == '__main__':
    main()
