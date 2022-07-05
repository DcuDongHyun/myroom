import matplotlib.pyplot as plt
import pyqtgraph
import pyqtgraph as pg
# import 로 불러온 pyqtgraph 의 별칭을 위한 as를 사용하여 pg로 해당 코드에서 재정의
import ros_numpy
#벡터 및 행렬 연산와 고성능의 수치 계산을 위한 ros용 numpy(Numerical Python) 모듈 호출
import sensor_msgs
# Ros master를 통한 센서 데이터 송 수신을 위한 모듈 호츌
"""import 는 모듈 호출하기 위한 방법 중 하나, 또 다른 방법은, from '모듈' import '모듈 내에서 필요한 기능' 만 호출 """
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import rosbag
import rospy
import time, random
from threading import Thread
import numpy as np
import mouse

""" ExMain(사용자가 지정한 이름)은 QWidget(모듈)의 기능 및 함수 사용을 위하여 QWidget 을 상속 받음 """
"""class -> 붕어빵 틀, 붕어빵 틀(class)에 다양한 재료를 넣어 각기 다른 붕어빵(Object) 생산 가능"""
"""객체 A : A를 단독으로 칭할 때 사용, 클래스에 의해 만들어진 객체는 인스턴스라고 부름 ex) b는 붕어빵 틀의 인스턴스 """


class ExMain(QWidget):
    def __init__(self):
        """def__init__(self)의 역할: 생성자 생성(초기화를 위해), 정의 :컨스트럭터라고 불리는 초기화를 위한 함수(메소드) """
        super().__init__()
        self.btn3 = True
        self.btn2 = True
        self.x = None
        self.y = None
        self.event=None
        self.key_event = None
        self.draw_pos = list()
        self.keycount = 1
        #직선의 양끝 점 좌표 값 리스트
        self.linedrawpoint = list()
        #직선 출력용
        self.draw = np.empty((0, 2),dtype='float64')
        self.adjcount = 0
        #인접한 직선을 표시하는 adj numpy배열
        self.adj = np.array([
                [0, 1],
            ])
        """super().__init__()의 의미 : 부모 클래스(super class)의 생성자를 쓴다는 의미"""
        #self.setMouseTracking(True)  # 마우스 트레킹(추적)을 위해 선언, True일떄는 모든 움직임 감시, False는 클릭 시 동
        hbox = QGridLayout()
        """일반적으로 컨터이너 속에 위젯을 넣어 구상함 이때 어떤식으로 위젯을 배치 할지 정하는 과정이 Layout(배치)"""
        """바둑판처럼 가로세로를 일정한 간격으로 직각이 되게 짠 구조나 물건. 또는 그런 형식. 흔히 생각하는 체크무늬 또한 격자"""
        self.canvas = pg.GraphicsLayoutWidget()
        # self.canvas = TestGraphicsWidget()
        """켄버스에 pg(PyqtGraph).GraphicsLayoutWiget()추가 """
        hbox.addWidget(self.canvas)
        """hbox(QGridLayout)에 canvas에 저장된 위젯 추가"""
        self.setLayout(hbox)
        """hbox에 설정된 Layout셋팅 하는 명령어"""
        # self.setGeometry(300, 100, 1000, 1000)  # x, y, width, height
        # canvas화면(scene)에서 마우스 클릭 시그널 발생 시 onClick 함수 호출(연결)
        self.canvas.scene().sigMouseClicked.connect(self.onClick)
        self.canvas.scene().sigMouseClicked.connect(self.delete)

        #점 삭제를 위한 버튼 추가
        self.btnRun = QPushButton("전체 삭제", self)
        self.btnRun.clicked.connect(self.allDelete)
        hbox.addWidget(self.btnRun, 1, 0)

        self.btnRun2 = QPushButton("선택 삭제", self)
        self.btnRun2.setCheckable(True)
        self.btnRun2.clicked.connect(self.btnClick2)
        self.btnRun2.toggle()
        hbox.addWidget(self.btnRun2, 1, 1)

        self.btnRun3 = QPushButton("점 생성", self)
        self.btnRun3.setCheckable(True)
        self.btnRun3.clicked.connect(self.btnClick)
        self.btnRun3.toggle()
        hbox.addWidget(self.btnRun3, 1, 2)

        self.btnRun4 = QPushButton("직선 삭제", self)
        hbox.addWidget(self.btnRun4, 1, 3)

        self.view = self.canvas.addViewBox()
        self.graph_item = pg.GraphItem()
        self.view.addItem(self.graph_item)
        """view = canvas에 viewbox를 추가한 객체"""
        self.view.setAspectLocked(True)
        """객체를 바라보는 각도(시야의 기울기)조정, (True로 직각 방향으로 잠금 설정, True.x(x=임의의 값)으로 기울기 변경 가능)"""
        self.view.disableAutoRange()
        """setAspectLocked 와 비슷 하며, 객체 전체가 화면에 나오는 것을 막기 위해(전체가 나올 시 화면 축소 현상을 막기 위하여) 선언, 기본 값 True """
        self.view.scaleBy(s=(20, 20))
        """화면의 격자(범위) 크기를 지정하는 함수(값이 증가 할수록 넓은 범위가 보임), 스케일링 범위 설정 시 disableAutoRange(False)으로 인한 자동 화면 축소 방지 """
        self.grid = pg.GridItem()
        """grid 객체에 pyqtgraph의 gridItem 선언"""
        self.view.addItem(self.grid)
        """view에 gridItem 추가"""
        # self.geometry().setWidth(1000)
        # self.geometry().setHeight(1000)
        self.setWindowTitle("realtime")
        # point cloud 출력용
        self.spt = pg.ScatterPlotItem(pen=pg.mkPen(width=1, color='r'), symbol='o', size=2)
        """scatter형태로 pg(파이썬 그래프), 에 추가할 내용 spt에 저장, pen의 형태는 너비 =1, 색은 red, 심볼(형태)는 동그라미, 크기는 2 """
        self.mpt = pg.ScatterPlotItem(pen=pg.mkPen(width=1, color='b'), symbol='o', size=10)
        self.ppt = pg.ScatterPlotItem(pen=pg.mkPen(width=1, color='g'), symbol='o', size=10)
        self.view.addItem(self.spt)
        self.view.addItem(self.mpt)
        self.view.addItem(self.ppt)
        """spt에 저장된 내용 뷰에 추가"""
        """QtGui 지원 2022.05 일자로 종료 따라서, QtGui -> QtWidgets 로 변경 """

        # global position to display graph
        self.pos = None
        """pos 선언후 초기화"""
        #마우스로 생성된 점 저장하기 위한 리스트
        self.mymousePoints = list()
        # object 출력용
        self.objs = list()  # for display to graph

        # object 출력용 position과 size
        self.objsPos = list()
        self.objsSize = list()
        # 출력용 object를 미리 생성해둠
        # 생성된 object의 position값을 입력하여 그래프에 출력할 수 있도록 함
        numofobjs = 50
        """for문 반복 횟수를 위한 numofobjs 선언후 50 할당"""
        for i in range(numofobjs):
            # obj = pg.QtGui.QGraphicsRectItem(-0.5, -0.5, 0.5, 0.5)  # obj 크기는 1m로 고정시킴
            # """obj을 선언 후 사각형 생성을 위한 함수 선언 (사각형의 크기 x= -0.5, y= -0.5, 너비 0.5분, 높이 0.5)"""
            # obj.setPen(pg.mkPen('w'))
            # """obj(사각형)에 사용할 pen 생성, 색상은 w(흰색)"""
            # self.view.addItem(obj)
            # """생성한 사각형 뷰 추가"""
            # self.objs.append(obj)
            # """objs에, obj에 저장된 사각형을 그래프에 나타내기 위해"""
            """여기 부터 point cloud 출력을 위한 구현 부"""
            pos = [0, 0, 0]  # x, y, z
            size = [0, 0, 0]  # w, h, depth
            self.objsPos.append(pos)
            self.objsSize.append(size)


        # load bagfile
        test_bagfile = '/home/kimdong/development/dataset/2022-01-26-16-28-58.bag'
        """bag 데이터 파일 경로 명시 및 객체에 추가"""
        self.bag_file = rosbag.Bag(test_bagfile)
        """bag 데이터 처리를 위해 rosbag.Bag에 넘겨준 후 그결과 bag_file에 저장 """
        # ros thread

        self.bagthreadFlag = True
        """bagthread의 상태를 확인하기 위한 bagthreadFlag 선언후 true 값 할당"""
        self.bagthread = Thread(target=self.getbagfile)
        """스레드 사용하기 위해 스레드 사용 타겟 지정"""
        self.bagthread.start()
        """스래드 시작(point cloud 출력을 하는 for 반복문에 대입.) """
        # Graph Timer 시작
        self.mytimer = QTimer()
        """ QTime란, PyQt에서 시간의 경과를 체크할 수 있는 함수"""
        self.mytimer.start(10)  # 1초마다 차트 갱신 위함...
        self.mytimer.timeout.connect(self.get_data) # 데이터 출력부분
        """timeout.connect의 역할 : 매 Interval마다 어떤 함수를 실행할지를 결정합니다. 만약, setInterval 함수로 Interval을 설정하지 않은 경우, 1ms마다 함수가 반복됩니다."""
        self.show()


    @pyqtSlot()
    def get_data(self):  # 벡 데이터 출력부분
        if self.pos is not None:
            """pos가 None이 아닐 때 """
            pass
            #self.spt.setData(pos=self.pos)  # line chart 그리기
            """spt{PointCloud를 화면에 표시 하기 위한 객체(색상 =r, 심볼 = o, 크기= 2)}에 Pos(x, y, z 좌표가 들어간 객체) 데이터 삽입"""


        # object 출력
        # 50개 object중 position 값이 0,0이 아닌것만 출력
        for i, obj in enumerate(self.objs):
            """enumerate = 인덱스(index)와 원소를 동시에 접근하기 위한 파이썬 for문 사용법 중 하나"""
            objpos = self.objsPos[i]
            
            """objspos= pos의 값이 들어간 배열(리스트)"""
            objsize = self.objsSize[i]
            """objsSize= size 값이 들어간 배열(리스트)"""
            if objpos[0] == 0 and objpos[1] == 0:
                """objpos의 0번 쨰와 1번째의 값이 0일 때"""
                obj.setVisible(False)
                """setVisible은 레이블의 가시성을 나타내기 위한 함수, True 일 때 레이블 표시 """
            else:
                obj.setVisible(True)
                """바로 위에 설명과 동일한 부분"""
                obj.setRect((objpos[0]) - (objsize[0] / 2), (objpos[1]) - (objsize[1] / 2), objsize[0], objsize[1])
                """setRect()= 사각형을 그리기 위함, setRect(x, y, w, h)의 형태로 사용 중 (위 코드 주석 처리 시 화면에 흰색 사각형 없어짐)"""
        # time.sleep(1)
        # print('test')짐



    # ros 파일에서 velodyne_points 메시지만 불러오는 부분
    def getbagfile(self):
        read_topic = '/velodyne_points'  # 메시지 타입

        for topic, msg, t in self.bag_file.read_messages(read_topic):
            if self.bagthreadFlag is False:
                break
            # ros_numpy 데이터 타입 문제로 class를 강제로 변경
            msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2

            # get point cloud
            pc = ros_numpy.numpify(msg)
            """ros_numpy.numpify = 메시지에서 numpy 배열을 추출하기 위한 변환 함수 모음"""
            points = np.zeros((pc.shape[0], 4))  # point배열 초기화 1번 컬럼부터 x, y, z, intensity 저장 예정

            # for ROS and vehicle, x axis is long direction, y axis is lat direction
            # ros 데이터는 x축이 정북 방향, y축이 서쪽 방향임, 좌표계 오른손 법칙을 따름
            points[:, 0] = pc['x']
            """[: ,0 ] 의 의미 : Numpy/Pandas에서 사용되는 표기법 이며, 배열의 전체 row의 첫번째 요소를 pc['x']로 업데이트 한다는 의미
            pc['x']의 x 는 키 값? pc에 저장된 """
            points[:, 1] = pc['y']
            points[:, 2] = pc['z']
            points[:, 3] = pc['intensity']
            """위 points 배열은 위 주석의 설명과 동일한 기능 """

            self.resetObjPos()
            """밑에 해당 함수 구현 되어있음 (의미 그대로 reset 하는 함수)"""
            self.doYourAlgorithm(points)
            """마찬가지로 밑에 해당 함수 구현되어 있"""
            # print(points)
            #time.sleep(0.1)  # 빨리 볼라면 주석처리 하면됨음
            """실행 시간에 딜레이 거는 함수"""


    # 여기부터 object detection 알고리즘 적용해 보면 됨
    def doYourAlgorithm(self, points):
        # downsampling

        # filter

        # obj  detection

        # 그래프의 좌표 출력을 위해 pos 데이터에 최종 points 저장
        self.pos = points


        # 테스트용 obj 생성, 임시로 0번째 obj에 x,y 좌표와 사이즈 입력
        tempobjPos = self.objsPos[0]
        tempobjSize = self.objsSize[0]
        tempobjPos[0] = 1
        tempobjPos[1] = 3
        tempobjSize[0] = 3
        tempobjSize[1] = 1.3


    def resetObjPos(self):
        for i, pos in enumerate(self.objsPos):
            # reset pos, size
            pos[0] = 0
            pos[1] = 0
            os = self.objsSize[i]
            os[0] = 0
            os[1] = 0

    def closeEvent(self, event):
        print('closed')
        self.bagthreadFlag = False

    # @pyqtSlot()
    # def mouseDoubleClickEvent(self, event):
    #     ##pyqt 창에서의 마우스 이벤트 처리 즉 pyqtgraph 마우스 좌표와 다름.
    #     self.x = event.x()
    #     self.y = event.y()
    #     floatx = np.float64(self.x)
    #     floaty = np.float64(self.y)
    #     print(floatx, floaty)

    #마우스 좌표에 점 찍고 위치 저장 및 점 유지
    def btnClick(self):
        self.btn3 = self.btnRun3.isChecked()
        self.btnRun3.setText("중지")
        if self.btn3 == True:
            self.btnRun3.setText("점 생성")

    def btnClick2(self):
        self.btn2 = self.btnRun2.isChecked()
        self.btnRun2.setText("선택 삭제 중지")
        if self.btn2 == True:
            self.btnRun2.setText("선택 삭제")

    #마우스 이벤트 저장하는 함수
    def onClick(self, event):
        key_event = self.key_event
        if(key_event == True and event.button() == 1):
            self.event = event
            conmousePoint = self.view.mapSceneToView(event._scenePos)
            point = np.array([[conmousePoint.x(), conmousePoint.y()]])
            self.draw = np.append(self.draw, point,axis=0)
            self.linedrawpoint = self.draw
            self.graph_item.setData(pos=self.draw, adj=self.adj)
            self.adjcount += 1
            self.adj = np.append(self.adj, np.array([[self.adjcount-1,self.adjcount]]), axis=0)
            print(self.adj)
        elif(event.button() == 1):
            self.event = event
            mousePoint = self.view.mapSceneToView(event._scenePos)
            if self.btn3 == False:
                # test_list는 setData.pos에 배열 or 튜플 형태를 넣어야 함
                test_list = [[mousePoint.x(), mousePoint.y()]]
                self.mymousePoints += test_list
                print("현제 마우스 좌표", self.mymousePoints)
                self.mpt.setData(pos=self.mymousePoints)

    def keyPressEvent(self, key_event):
        if(key_event.modifiers() & Qt.ControlModifier):
            self.keycount += 1
            if self.keycount % 2 == 0:
                key_event = True
                self.key_event = key_event
            else:
                key_event = False
                self.key_event = key_event


    #점 전체 삭제 함수
    def allDelete(self):
        self.mymousePoints = []
        self.mpt.setData(pos=self.mymousePoints)

    #점 선탣 삭제 함수
    def delete(self, event):
        self.event = event
        mousePoint = self.view.mapSceneToView(event._scenePos)
        if self.btn2 == False:
            test2_list = [[mousePoint.x(), mousePoint.y()]]
            test3_list = [[int(i[0]), int(i[1])] for i in test2_list]
            test4_list = [[int(i[0]), int(i[1])] for i in self.mymousePoints]
            self.test4_list = test4_list
            for i, data in enumerate(test4_list):
                last_pos = test3_list[0]
                if data == last_pos:
                    del self.mymousePoints[i]
                    self.mpt.setData(pos=self.mymousePoints)

    # def EuclideanDistance(self):
    #     event = self.event
    #     mousePoint = self.view.mapSceneToView(event._scenePos)
    #     test_list = [[mousePoint.x(), mousePoint.y()]]


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)

    ex = ExMain()

    sys.exit(app.exec_())