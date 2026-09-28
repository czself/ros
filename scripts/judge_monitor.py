#!/usr/bin/env python3
"""Read-only live competition console, paired with source-stamped ROS images."""
from collections import OrderedDict, deque
import json
from pathlib import Path
import sys
import threading
import unicodedata
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

NAMES={'resident':('人偶','社区人员'),'stranger':('人偶','外来人员'),
       'red_on':('红绿灯','红灯亮'),'red_off':('红绿灯','红灯灭'),
       'yellow_on':('红绿灯','黄灯亮'),'yellow_off':('红绿灯','黄灯灭'),
       'green_on':('红绿灯','绿灯亮'),'green_off':('红绿灯','绿灯灭'),
       'license_plate':('车牌检测','车牌区域')}
STATES={'RED':'红灯','GREEN':'绿灯','YELLOW':'黄灯','UNKNOWN':'未知'}

def padded(text,width):
    size=sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text)
    return text+' '*max(0,width-size)

def clipped(text,width):
    result='';size=0
    for character in text:
        amount=2 if unicodedata.east_asian_width(character) in 'WF' else 1
        if size+amount>width:return result+'…'
        result+=character;size+=amount
    return result

class JudgeMonitor:
    def __init__(self):
        self.directory=Path(rospy.get_param('~photo_dir'))/'judge'
        self.directory.mkdir(parents=True,exist_ok=True)
        self.log=(self.directory/'recognition_terminal.txt').open('a',encoding='utf-8',buffering=1)
        self.events=(self.directory/'recognition_events.jsonl').open('a',encoding='utf-8',buffering=1)
        self.lock=threading.RLock();self.bridge=CvBridge()
        self.records={k:OrderedDict() for k in ('yolo','ocr','person')}
        self.images={k:OrderedDict() for k in self.records}
        self.publishers={k:rospy.Publisher('/inspection/judge_'+k+'_image',Image,queue_size=1,latch=True) for k in self.records}
        self.event_pub=rospy.Publisher('/inspection/judge_event',String,queue_size=10)
        self.display_pub=rospy.Publisher('/inspection/judge_display_image',Image,queue_size=1,latch=True)
        self.ocr_hold_until=0.0
        self.sequence=0;self.done=False;self.latest={};self.plates={};self.people={}
        self.status='准备中';self.goal='HOME';self.gate='等待门控信息';self.final_counts=None
        self.history=deque(maxlen=7)
        rospy.Subscriber('/inspection/detections',String,lambda m:self.record('yolo',m),queue_size=3)
        rospy.Subscriber('/inspection/plate_report',String,lambda m:self.record('ocr',m),queue_size=3)
        rospy.Subscriber('/inspection/person_report',String,lambda m:self.record('person',m),queue_size=3)
        for kind,topic in [('yolo','/inspection/image'),('ocr','/inspection/plate_image'),('person','/inspection/person_image')]:
            rospy.Subscriber(topic,Image,lambda m,k=kind:self.image(k,m),queue_size=2,buff_size=16*1024*1024)
        rospy.Subscriber('/route/status',String,self.route_status,queue_size=1)
        rospy.Subscriber('/route/progress',String,self.progress,queue_size=1)
        rospy.Subscriber('/traffic_light/gate_status',String,lambda m:setattr(self,'gate',m.data),queue_size=1)
        rospy.Timer(rospy.Duration(.35),self.refresh)

    @staticmethod
    def key(stamp):return int(stamp['secs']),int(stamp['nsecs'])

    @staticmethod
    def put(cache,key,value):
        cache[key]=value
        while len(cache)>8:cache.popitem(last=False)

    def record(self,kind,message):
        data=json.loads(message.data)
        with self.lock:
            if kind=='person' and 'waypoint' not in data:
                self.final_counts=data
                self.write_lines(['[街区统计] 类别=全街区人数统计 内容='+data['speech_text']])
                return
            if self.done:return
            key=self.key(data['source_stamp']);self.put(self.records[kind],key,data);self.match(kind,key)

    def image(self,kind,message):
        with self.lock:
            if self.done:return
            key=(message.header.stamp.secs,message.header.stamp.nsecs)
            self.put(self.images[kind],key,message);self.match(kind,key)

    def match(self,kind,key):
        if key not in self.records[kind] or key not in self.images[kind]:return
        record=self.records[kind].pop(key);message=self.images[kind].pop(key)
        self.sequence+=1;sequence=self.sequence;stamp=key[0]+key[1]*1e-9;rows=[]
        if kind=='yolo':
            self.latest=record;targets=record['detections']
            for i,target in enumerate(targets,1):
                category,content=NAMES.get(target['class'],('目标',target['class']))
                rows.append('[%04d-%02d YOLO] 帧=%.9f 类别=%s 内容=%s 名称=%s 置信度=%.3f 框=%s' % (
                    sequence,i,stamp,category,content,target['class'],target['confidence'],target['box']))
            rows.append('[%04d 灯态] 帧=%.9f 类别=红绿灯状态 内容=%s 置信度=%.3f' % (
                sequence,stamp,STATES.get(record['traffic_state'],record['traffic_state']),record['traffic_confidence']))
        elif kind=='ocr':
            self.plates[record['waypoint']]=record
            selected=next(f for f in record['frames'] if f['source_stamp']==record['source_stamp'])
            targets=[dict(selected,confidence=record['confidence'],content=record['text'],**{'class':'license_plate'})]
            rows.append('[%04d-01 OCR] 帧=%.9f 类别=车牌字符 内容=%s 置信度=%.3f 点位=%s 一致帧=%d' % (
                sequence,stamp,record['text'] or '未识别',record['confidence'],record['waypoint'],record['matching_frames']))
        else:
            targets=record['people']
            for i,p in enumerate(targets,1):
                self.people[p['person_id']]=p
                rows.append('[%04d-%02d 人偶] 帧=%.9f 类别=人偶 内容=%s 编号=%s 街区=%s 置信度=%.3f' % (
                    sequence,i,stamp,NAMES[p['class']][1],p['person_id'],p['street'],p['confidence']))
        annotated=self.bridge.imgmsg_to_cv2(message,'bgr8').copy();scale=annotated.shape[1]/640.0
        cv2.putText(annotated,'%s #%04d source %.9f' % (kind.upper(),sequence,stamp),(8,int(40*scale)),
                    cv2.FONT_HERSHEY_SIMPLEX,.48*scale,(255,255,255),max(1,int(scale)),cv2.LINE_AA)
        for i,target in enumerate(targets,1):
            x1,y1,_,_=map(int,target['box'])
            cv2.putText(annotated,'#%02d'%i,(max(0,x1+2),max(12,y1+int(14*scale))),
                        cv2.FONT_HERSHEY_SIMPLEX,.4*scale,(255,255,255),max(1,int(scale)),cv2.LINE_AA)
        output=self.bridge.cv2_to_imgmsg(annotated,'bgr8');output.header=message.header;output.header.seq=sequence
        self.publishers[kind].publish(output)
        now=rospy.Time.now().to_sec()
        if kind=='ocr':
            self.ocr_hold_until=now+3.0
            self.display_pub.publish(output)
        elif kind=='person' or (kind=='yolo' and now>=self.ocr_hold_until):
            self.display_pub.publish(output)
        event={'sequence':sequence,'kind':kind,'source_stamp':record['source_stamp'],'targets':targets,
               'terminal_lines':rows,'image_topic':'/inspection/judge_'+kind+'_image'}
        self.events.write(json.dumps(event,ensure_ascii=False)+'\n');self.event_pub.publish(String(data=json.dumps(event,ensure_ascii=False)))
        self.write_lines(rows)

    def write_lines(self,lines):
        for line in lines:
            self.log.write(line+'\n');self.history.append(line)
            if not sys.stdout.isatty():print(line,flush=True)

    def progress(self,message):self.goal=json.loads(message.data).get('goal',self.goal)

    def route_status(self,message):
        self.status=message.data
        if 'COMPLETE_PARKED' in message.data or message.data.startswith('FAILED'):self.done=True

    def refresh(self,_):
        with self.lock:
            if not sys.stdout.isatty():return
            rec=self.latest
            left=['YOLO 实时识别：图像和文字以相同源帧配对',
                  '源帧 %.9f | 延迟 %.1fms | 帧龄 %.1fms' % (rec.get('source_stamp',{}).get('seconds',0),rec.get('latency_ms',0),rec.get('frame_age_ms',0))]
            for i,t in enumerate(rec.get('detections',[])[:8],1):
                category,content=NAMES.get(t['class'],('目标',t['class']))
                left.append('#%02d 类别=%s 内容=%s [%s] 置信度=%.3f' % (i,category,content,t['class'],t['confidence']))
            left+=['','红绿灯：'+STATES.get(rec.get('traffic_state','UNKNOWN'),'未知'),'通行门控：'+self.gate,
                   '','最近识别顺序（完整文字逐项写入 recognition_terminal.txt）']+[clipped(x,112) for x in list(self.history)[-5:]]
            right=['OCR 车牌字符识别（当前任务真实数据）']
            for point in ['POINT_8','POINT_9','POINT_10']:
                p=self.plates.get(point)
                right.append('%s：%s' % (point,('%s 置信%.3f %d帧一致' % (p['text'] or '未识别',p['confidence'],p['matching_frames'])) if p else '尚未到达'))
            streets={s:{'total':0,'resident':0,'stranger':0} for s in ['A','B']}
            for p in self.people.values():streets[p['street']]['total']+=1;streets[p['street']][p['class']]+=1
            counts={'total':len(self.people),'resident':sum(p['class']=='resident' for p in self.people.values()),'stranger':sum(p['class']=='stranger' for p in self.people.values())}
            if self.final_counts:counts=self.final_counts['counts'];streets=self.final_counts['street_counts']
            right+=['','整个街区统计（唯一人员编号去重）','总人数%d | 社区人员%d | 外来人员%d' % (counts['total'],counts['resident'],counts['stranger'])]
            for street,c in streets.items():right.append('%s街区：%d人，社区%d，外来%d' % (street,c['total'],c['resident'],c['stranger']))
            right+=['外来人员：'+', '.join(p['person_id']+'('+p['street']+')' for p in self.people.values() if p['class']=='stranger'),
                    '','来源：best.pt / PP-OCRv5_mobile_rec / RGB+深度+TF']
            lines=['智算三行队 · 智慧社区比赛演示 · 真实识别数据 · '+self.directory.parent.name,
                   '任务状态：%s | 当前点位：%s | 已配对显示：%d帧' % (self.status,self.goal,self.sequence),'='*205]
            for i in range(max(len(left),len(right))):lines.append(padded(left[i] if i<len(left) else '',116)+' | '+(right[i] if i<len(right) else ''))
            sys.stdout.write('\033[2J\033[H'+'\n'.join(lines)+'\n');sys.stdout.flush()
            state={'route_status':self.status,'goal':self.goal,'frame_pairs':self.sequence,'people_counts':counts,
                   'street_counts':streets,'ocr':{k:v['text'] for k,v in self.plates.items()},'gate_status':self.gate,'traffic_state':rec.get('traffic_state','UNKNOWN')}
            (self.directory/'dashboard_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':
    rospy.init_node('judge_monitor');JudgeMonitor();rospy.spin()
