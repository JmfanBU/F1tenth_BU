;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::BoundingBoxes)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'BoundingBoxes (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::BOUNDINGBOXES")
  (make-package "DARKNET_ROS_MSGS::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html


(defclass darknet_ros_msgs::BoundingBoxes
  :super ros::object
  :slots (_boundingBoxes ))

(defmethod darknet_ros_msgs::BoundingBoxes
  (:init
   (&key
    ((:boundingBoxes __boundingBoxes) (let (r) (dotimes (i 0) (push (instance darknet_ros_msgs::BoundingBox :init) r)) r))
    )
   (send-super :init)
   (setq _boundingBoxes __boundingBoxes)
   self)
  (:boundingBoxes
   (&rest __boundingBoxes)
   (if (keywordp (car __boundingBoxes))
       (send* _boundingBoxes __boundingBoxes)
     (progn
       (if __boundingBoxes (setq _boundingBoxes (car __boundingBoxes)))
       _boundingBoxes)))
  (:serialization-length
   ()
   (+
    ;; darknet_ros_msgs/BoundingBox[] _boundingBoxes
    (apply #'+ (send-all _boundingBoxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; darknet_ros_msgs/BoundingBox[] _boundingBoxes
     (write-long (length _boundingBoxes) s)
     (dolist (elem _boundingBoxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; darknet_ros_msgs/BoundingBox[] _boundingBoxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _boundingBoxes (let (r) (dotimes (i n) (push (instance darknet_ros_msgs::BoundingBox :init) r)) r))
     (dolist (elem- _boundingBoxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::BoundingBoxes :md5sum-) "ea71eb1791dc15f22ef61ff50be3c0f1")
(setf (get darknet_ros_msgs::BoundingBoxes :datatype-) "darknet_ros_msgs/BoundingBoxes")
(setf (get darknet_ros_msgs::BoundingBoxes :definition-)
      "BoundingBox[] boundingBoxes

================================================================================
MSG: darknet_ros_msgs/BoundingBox
string Class
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax


")



(provide :darknet_ros_msgs/BoundingBoxes "ea71eb1791dc15f22ef61ff50be3c0f1")


