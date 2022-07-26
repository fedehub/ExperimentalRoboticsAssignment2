(define (problem task)
(:domain db_domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    tp - temple
)
(:init

    (has_been_at wp2)
    (has_been_at wp3)
    (has_been_at wp4)



    (consistent_hypo)

    (at_temple tp)

    (gathered_hint wp1)
    (gathered_hint wp2)
    (gathered_hint wp3)
    (gathered_hint wp4)


    (not_has_been_at wp4)

    (not_gripper_inplace)

)
(:goal (and
    (true_hypo)
))
)
