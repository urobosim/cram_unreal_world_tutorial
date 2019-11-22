;;;
;;; Copyright (c) 2017, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :urw-tut)

(defparameter list-of-objects
  '(:milk))

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(defparameter *object-colors*
  '((:spoon . "blue")))

(defun get-kitchen-urdf ()
  (slot-value
    (btr:object btr:*current-bullet-world* :kitchen)
    'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                                (joint-angle 0.2d0) (kitchen-name :kitchen))
  (btr:set-robot-state-from-joints
    `((,joint-name  ,joint-angle))
    (btr:object btr:*current-bullet-world* kitchen-name)))

(defparameter *pose-1*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.19 -1.07 0)
    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))))

(defparameter *pose-2*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.4 0.9 0)
    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))))

(defparameter *pose-3*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.4 0.9 0)
    (cl-transforms:make-identity-rotation)))

(defparameter *pose-4*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.8 0.9 0)
    (cl-transforms:make-identity-rotation)))

(defparameter *pose-5*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.4 0.9 0)
    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))))

(defparameter *pose-6*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.1 0.9 0)
    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))))

(defparameter *pose-7*
  (cl-transforms-stamped:make-pose-stamped
    "map" 0.0
    (cl-transforms:make-3d-vector 0.4 0.9 0)
    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))))

; (defun navigate-to (?navigation-goal)
;   (exe:perform (desig:a motion
;                         (type going)
;                         (target (desig:a location (pose ?navigation-goal))))))



(defparameter *object-fetching-location*
   (cl-transforms-stamped:make-pose-stamped
     "map" 0.0
     (cl-transforms:make-3d-vector 1.30 0.0 0.90)
     (cl-transforms:make-quaternion 0 0 0.6 0.4))
  )

(defparameter *object-placing-location*
   (cl-transforms-stamped:make-pose-stamped
     "map" 0.0
     (cl-transforms:make-3d-vector -0.5 0.8 0.95)
     (cl-transforms:make-quaternion 0 0 0.6 0.4))
  )



(defun navigate-to (?navigation-goal)
  (exe:perform
    (desig:an action
              (type going)
              (target (desig:a location
                               (pose ?navigation-goal))))))

(defun perceive-object (?object-type)
  (exe:perform (desig:a motion (type detecting)
                        (object (desig:an object (type ?object-type))))))

(defun look-at (?point-of-interest)
  (exe:perform (desig:a motion
                        (type looking)
                        (target (desig:a location (pose ?point-of-interest))))))

(defun get-perceived-bottle-desig ()
  (let* ((?bottle-desig (desig:an object (type bottle)))
         (?perceived-bottle-desig (exe:perform
                                    (desig:a motion
                                             (type detecting)
                                             (object ?bottle-desig)))))
    ?perceived-bottle-desig))

(defun park-arm (?right-configuration ?left-configuration)
  (exe:perform
    (desig:a motion
             (type moving-arm-joints)
             (left-configuration ?left-configuration)
             (right-configuration ?right-configuration))))


(defun pick-up (?object-designator &optional (?arm :right))
  (exe:perform (desig:an action
                         (type picking-up)
                         (arm ?arm)
                         (object ?object-designator))))

(defun place-down (?pose ?object ?arm)
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm)
                         (object ?object)
                         (target (desig:a location (pose ?pose))))))

(defun test-switch-two-bottles ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
                                    (cpl:top-level
                                      ;; Go to counter top and perceive bottle
                                      (let ((?navigation-goal *pose-1*)
                                            (?ptu-goal
                                              (cl-transforms-stamped:make-pose-stamped
                                                "base_footprint"
                                                0.0
                                                (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
                                                (cl-transforms:make-identity-rotation))))
                                        (cpl:par
                                          ;; Move torso up
                                          (navigate-to ?navigation-goal))
                                        )
                                      ;; Pick up bottle-1 with right arm.
                                      )))


; (defun test ()
;   (pr2-pms:with-real-robot
;
;     (object-placing-locations
;           (let ((?pose
;                   (cl-transforms-stamped:make-pose-stamped
;                    "map"
;                    0.0
;                    (cl-transforms:make-3d-vector -0.78 0.8 0.95)
;                    (cl-transforms:make-quaternion 0 0 0.6 0.4))))
;             `((:breakfast-cereal . ,(desig:a location
;                                              (pose ?pose)
;                                              ;; (left-of (an object (type bowl)))
;                                              ;; (far-from (an object (type bowl)))
;                                              ;; (for (an object (type breakfast-cereal)))
;                                              ;; (on (desig:an object
;                                              ;;               (type counter-top)
;                                              ;;               (urdf-name kitchen-island-surface)
;                                              ;;               (owl-name "kitchen_island_counter_top")
;                                              ;;               (part-of kitchen)))
;                                              ;; (side back)
;                                              ))
;               (:cup . ,(desig:a location
;                                 (right-of (an object (type bowl)))
;                                 ;; (behind (an object (type bowl)))
;                                 (near (an object (type bowl)))
;                                 (for (an object (type cup)))))
;               (:bowl . ,(desig:a location
;                                  (on (desig:an object
;                                                (type counter-top)
;                                                (urdf-name kitchen-island-surface)
;                                                (owl-name "kitchen_island_counter_top")
;                                                (part-of kitchen)))
;                                  (context table-setting)
;                                  (for (an object (type bowl)))
;                                  (object-count 3)
;                                  (side back)
;                                  (side right)
;                                  (range-invert 0.5)))
;               (:spoon . ,(desig:a location
;                                   (right-of (an object (type bowl)))
;                                   (near (an object (type bowl)))
;                                   (for (an object (type spoon)))))
;               (:milk . ,(desig:a location
;                                  (left-of (an object (type bowl)))
;                                  (far-from (an object (type bowl)))
;                                  (for (an object (type milk))))))))
;
;
;
;     (exe:perform (desig:a motion (type detecting)
;                           (object (desig:an object (type milk)))))
;
;     (?delivering-location
;       (cdr (assoc ?object-type object-placing-locations)))
;
;     (?object-to-fetch
;       (desig:an object
;                 (type milk)
;                 (location ?fetching-location)
;                 ))
;
;     (exe:perform
;       (desig:an action
;                 (type transporting)
;                 (object ?object-to-fetch)
;                 ;; (arm right)
;                 (location ?fetching-location)
;                 (target ?delivering-location)))
;
;     ; let
;     ; (
;     ;
;     ;  )
;
;     ; (exe:perform (desig:an action (type opening-gripper) (gripper right)))
;     ; (let
;     ;   ((cpl:par
;     ;      (exe:perform
;     ;        (desig:an action
;     ;                  (type positioning-arm)
;     ;                  (left-configuration park)
;     ;                  (right-configuration park))))))
;     ; (let
;     ;    ((?pose (cl-transforms-stamped:make-pose-stamped
;     ;              "base_footprint" 0.0
;     ;              (cl-transforms:make-3d-vector 0.5 0 0.9)
;     ;              (cl-transforms:make-identity-rotation))))
;     ;    (exe:perform
;     ;      (desig:an action
;     ;                (type looking)
;     ;                (target (desig:a location (pose ?pose))))))
;
;     ; (let
;       ; (cpl:par
;       ;   (navigate-to *pose-1*))
;       ; (cpl:par
;       ;   (navigate-to *pose-2*))
;       ; (cpl:par
;       ;   (navigate-to *pose-3*))
;       ; (cpl:par
;       ;   (navigate-to *pose-4*))
;       ; (cpl:par
;       ;   (navigate-to *pose-5* ))
;       ; (cpl:par
;       ;   (navigate-to *pose-6* ))
;       ; (cpl:par
;       ;   (navigate-to *pose-7* ))
;     ; Pick up bottle-1 with right arm.
;     ))

