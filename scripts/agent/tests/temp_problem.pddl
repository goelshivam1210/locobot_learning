(define (problem recycle) (:domain recycle_bot)
(:objects 
    doorway_1 - doorway
    room_1 room_2 - room
    ball_1 - ball
    can_1 - can
    bin_1 - bin
    nothing - nothing
    robot_1 - robot
)

(:init
    (connect room_1 room_2 doorway_1)
    (connect room_2 room_1 doorway_1)
    (at room_1 doorway_1)
    (at room_1 ball_1)
    (at room_1 can_1)
    (at room_2 bin_1)
    (at room_1 robot_1)
    (facing nothing)
    (hold nothing)
)

(:goal (and
    (contain ball_1 bin_1)
))
)