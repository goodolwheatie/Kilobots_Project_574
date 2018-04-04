#define SIMULATOR


#ifndef SIMULATOR
   #include <kilolib.h>
    #include <avr/io.h>  // for microcontroller register defs
    #include "ring.h"
    USERDATA myData;
    USERDATA *mydata = &myData;
#else
    #include <math.h>
    #include <kilombo.h>
    #include <stdio.h> // for printf
    #include "ring.h"
    REGISTER_USERDATA(USERDATA)
#endif

void send_joining();


char isQueueFull()
{
    return (mydata->tail +1) % QUEUE == mydata->head;
}


/* Helper function for setting motor speed smoothly
 */
void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
    // OCR2A = ccw;  OCR2B = cw;
#ifdef KILOBOT
    uint8_t l = 0, r = 0;
    if (ccw && !OCR2A) // we want left motor on, and it's off
        l = 0xff;
    if (cw && !OCR2B)  // we want right motor on, and it's off
        r = 0xff;
    if (l || r)        // at least one motor needs spin-up
    {
        set_motors(l, r);
        delay(15);
    }
#endif
    // spin-up is done, now we set the real value
    set_motors(ccw, cw);
}


void set_motion(motion_t new_motion)
{
    switch(new_motion) {
        case STOP:
            smooth_set_motors(0,0);
            break;
        case FORWARD:
            smooth_set_motors(kilo_straight_left, kilo_straight_right);
            break;
        case LEFT:
            smooth_set_motors(kilo_turn_left, 0);
            break;
        case RIGHT:
            smooth_set_motors(0, kilo_turn_right); 
            break;
    }
}


char in_interval(uint8_t distance)
{
    //if (distance >= 40 && distance <= 60)
    if (distance <= 90)
        return 1;
    return 0;
}


char is_stabilized()
{
    uint8_t i=0,j=0;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        
        if ((mydata->nearest_neighbors[i].state == AUTONOMOUS && mydata->nearest_neighbors[i].num > 2) ||
            (mydata->nearest_neighbors[i].state == COOPERATIVE && mydata->nearest_neighbors[i].num_cooperative > 2))
            j++;
    }

    return j == mydata->num_neighbors;
}


// Search for id in the neighboring nodes
uint8_t exists_nearest_neighbor(uint8_t id)
{
    uint8_t i;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        if (mydata->nearest_neighbors[i].id == id)
            return i;
    }
    return i;
}


// Search for id in the neighboring nodes
uint8_t are_all_cooperative()
{
    uint8_t i;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        if (mydata->nearest_neighbors[i].state == COOPERATIVE)
            return 0;
    }
    return 1;
}


uint8_t get_nearest_two_neighbors()
{
    uint8_t i, l, k;
    uint16_t min_sum = 0xFFFF;
    
    k = i = mydata->num_neighbors;
    if (are_all_cooperative())
    {
        for (i=0; i<mydata->num_neighbors; i++)
        {
            // shortest
            if (mydata->nearest_neighbors[i].distance < min_sum)
            {
                k = i;
            }
        }
        if (k < mydata->num_neighbors)
        {
            i = k;
        }
    }
    else
    {
        for (i=0; i<mydata->num_neighbors; i++)
        {
            // Is it cooperative and at distance in [4cm,6cm]?
            if (mydata->nearest_neighbors[i].state == COOPERATIVE)
            {
                l = exists_nearest_neighbor(mydata->nearest_neighbors[i].right_id);
                // Does the right exits in my table?
                if (l < mydata->num_neighbors)
                {
                    if (mydata->nearest_neighbors[i].distance +
                        mydata->nearest_neighbors[l].distance < min_sum)
                    {
                        min_sum = mydata->nearest_neighbors[i].distance + mydata->nearest_neighbors[l].distance;
                        k = i;
                    }
                }
            }
        }
        if (k < mydata->num_neighbors)
        {
            i = k;
        }
    }
    return i;
}


void recv_sharing(uint8_t *payload, uint8_t distance)
{
    if (payload[ID] == mydata->my_id  || payload[ID] == 0 || !in_interval(distance) ) return;
    
    uint8_t i = exists_nearest_neighbor(payload[ID]);
    if (i >= mydata->num_neighbors) // The id has never received
    {
        if (mydata->num_neighbors < MAX_NUM_NEIGHBORS)
        {
            i = mydata->num_neighbors;
            mydata->num_neighbors++;
            mydata->nearest_neighbors[i].num = 0;
            
        }
    }

    mydata->nearest_neighbors[i].id = payload[ID];
    mydata->nearest_neighbors[i].right_id = payload[RIGHT_ID];
    mydata->nearest_neighbors[i].left_id = payload[LEFT_ID];
    mydata->nearest_neighbors[i].state = payload[STATE];
    mydata->nearest_neighbors[i].distance = distance;
    // Testing
    mydata->nearest_neighbors[i].not_in_mis = payload[NOT_IN_MIS];
    mydata->nearest_neighbors[i].in_mis = payload[IN_MIS];
    mydata->nearest_neighbors[i].message_received = 1;              // Toggle the message_received boolean as true when a message is shared from this ID.
    if (payload[STATE] == AUTONOMOUS)
    {
        mydata->nearest_neighbors[i].num++;
        mydata->nearest_neighbors[i].num_cooperative = 0;
    }
    else
    {
        mydata->nearest_neighbors[i].num_cooperative++;
        mydata->nearest_neighbors[i].num = 0;
    }

}


void recv_move(uint8_t *payload)
{
/*#ifdef SIMULATOR
    printf("%d Receives move %d %d %d\n", mydata->my_id, payload[MSG], mydata->my_id, payload[RECEIVER]);
#endif*/
    
    if (mydata->my_id == payload[RECEIVER])
    {
        mydata->token  = 1;
        //mydata->blue  = 1;
        mydata->send_token = mydata->now + TOKEN_TIME * 4.0;

    }
   /* else if (my_id == payload[SENDER])
    {
        mydata->motion_state = STOP;
    }
    else
    {
        mydata->msg.data[MSG]      = payload[MSG];
        mydata->msg.data[ID]       = mydata->my_id;
        mydata->msg.data[RECEIVER] = payload[RECEIVER];
        mydata->msg.data[SENDER]   = payload[SENDER];
        mydata->msg.type           = NORMAL;
        mydata->msg.crc            = message_crc(&msg);
        mydata->message_sent       = 0;
    } */
}


void message_rx(message_t *m, distance_measurement_t *d)
{
    uint8_t dist = estimate_distance(d);
    
    if (m->type == NORMAL && m->data[MSG] !=NULL_MSG)
    {
        
/*#ifdef SIMULATOR
        printf("%d Receives %d %d\n", mydata->my_id,  m->data[MSG], m->data[RECEIVER]);
#endif*/
   
        recv_sharing(m->data, dist);
        switch (m->data[MSG])
        {
            case JOIN:
                // recv_joining(m->data);
                break;
            case MOVE:
                recv_move(m->data);
                break;
        
        }
    }
}


char enqueue_message(uint8_t m)
{
#ifdef SIMULATOR
 //   printf("%d, Prepare %d\n", mydata->my_id, m);
#endif
    if (!isQueueFull())
    {
        mydata->message[mydata->tail].data[MSG] = m;
        mydata->message[mydata->tail].data[ID] = mydata->my_id;
        mydata->message[mydata->tail].data[RIGHT_ID] = mydata->my_right;
        mydata->message[mydata->tail].data[LEFT_ID] = mydata->my_left;
        mydata->message[mydata->tail].data[RECEIVER] = mydata->my_right;
        mydata->message[mydata->tail].data[SENDER] = mydata->my_id;
        mydata->message[mydata->tail].data[STATE] = mydata->state;

        mydata->message[mydata->tail].data[IN_MIS] = mydata->in_mis;
        mydata->message[mydata->tail].data[NOT_IN_MIS] = mydata->not_in_mis;
    
        mydata->message[mydata->tail].type = NORMAL;
        mydata->message[mydata->tail].crc = message_crc(&mydata->message[mydata->tail]);
        mydata->tail++;
        mydata->tail = mydata->tail % QUEUE;
        return 1;
    }
    return 0;
}

/**********************************/
/**********************************/
void send_joining()
{
    uint8_t i;
    /* precondition  */
    
    if (mydata->state == AUTONOMOUS && is_stabilized()  && !isQueueFull())
    {

        i = get_nearest_two_neighbors();
        if (i < mydata->num_neighbors && mydata->message_sent == 1)
        {
            // effect:
            // Added color to the join sender.
            mydata->blue = 255;
            mydata->green = 255;
            mydata->red = 255;

            mydata->state = COOPERATIVE;
            mydata->my_right = mydata->nearest_neighbors[i].right_id;
            mydata->my_left = mydata->nearest_neighbors[i].id;

            enqueue_message(JOIN);
#ifdef SIMULATOR
            printf("Sending joining - ");
            // print_state();  
#endif
        }
    }
}

void send_sharing()
{
    // Precondition
    if (mydata->now >= mydata->nextShareSending  && !isQueueFull())
    {
        // Sending
        enqueue_message(SHARE);
        // effect:
        mydata->nextShareSending = mydata->now + SHARING_TIME;
    }
}


void send_move()
{
    // Precondition:
    if (mydata->state == COOPERATIVE  && mydata->token )
    {
        mydata->send_token = mydata->now + TOKEN_TIME;
    }
    if (mydata->state == COOPERATIVE && !isQueueFull() && mydata->token && mydata->send_token <= mydata->now)
    {
            // Sending
        enqueue_message(MOVE);
        mydata->token = 0;
        //mydata->blue = 0;
        // effect:
    }

}

void move(uint8_t tick)
{
    // Precondition:
    if (mydata->motion_state == ACTIVE && mydata->state == COOPERATIVE)
    {
        
      /*  if (mydata->time_active == mydata->move_motion[mydata->move_state].time)
        {
            // Effect:
            mydata->green = 1;
            mydata->move_state++;
            if (mydata->move_state == 3)
            {
                mydata->send_token = 1;
                send_move();
#ifdef SIMULATOR
                printf("Sending Move %d\n", mydata->my_id);
#endif
                mydata->motion_state = STOP;
                return;
            }
            mydata->time_active = 0;
        
        }
        set_motion(mydata->move_motion[mydata->move_state].motion);
        mydata->time_active++;
       */
    }
    else
    {
        mydata->green = 0;
        set_motion(STOP);
    }
    
}

/**
 * Resets the data state and color scheme.
 **/
void reset_data()
{
    mydata->state = AUTONOMOUS;
    mydata->my_left = mydata->my_right = mydata->my_id;
    mydata->num_neighbors = 0;
    mydata->time_active = 0;
    mydata->red = 0;
    mydata->green = 0;
    mydata->blue = 0;
    mydata->loop_counter = 0;
    mydata->round_counter = 0;
    mydata->not_in_mis = 0;
    mydata->in_mis = 0;
}

char check_message_received()
{
    uint8_t i;

    for(i = 0; i < mydata->num_neighbors; i++)
    {
        if (mydata->nearest_neighbors[i].message_received == 0)     // Found a neighbor that's missing a message.
        {
            /*#ifdef SIMULATOR
                printf("Missing message (ID): %d\n", mydata->nearest_neighbors[i].id);  // Debugging text
            #endif*/
            return 0;
        }
    }
    return 1;
}

void reset_receive_checker()
{
    uint8_t i;

    mydata->loop_counter = 0;
    for(i = 0; i < mydata->num_neighbors; i++)
    {
        mydata->nearest_neighbors[i].message_received = 0;
    }
}

void check_messages()
{
    /*#ifdef SIMULATOR
        printf("Message received: %i, Loop Counter: %d, Now: %d\n", check_message_received(), mydata->loop_counter, mydata->now);     // Debugging text.
    #endif*/
    if (mydata->num_neighbors == 0)
    {
        // Do nothing
    }
    else if (check_message_received())      // A message from each neighbor has been received.
    {
        reset_receive_checker();
    }
    else if (mydata->loop_counter >= 100)    // Some neighbors are missing and loop timeout is reached
    {
        reset_data();
        reset_receive_checker();
    }
    else                                    // Some neighbors are missing but we haven't reached loop timeout.
        mydata->loop_counter++;
}

// second check function in case that a lower id neighbor didn't join the mis because it had another 
// neighbor, that is not a neighbor to the current kilobot, that was in the mis.
void check_if_no_MIS_nearby()
{
    uint8_t index = 0;;
    while (index < mydata->num_neighbors)
    {
        if (mydata->nearest_neighbors[index].not_in_mis != 1)
        {
            mydata->not_in_mis = 1;
        }
        ++index;
    }
    if (mydata->not_in_mis == 2)
        mydata->in_mis = 1;
}


/*
    flag 1 for not_in_mis is for if you did not enter the mis because of a neighbors that is already in the mis.
    flag 2 for not_in_mis is for if you did not enter the mis because your number was bigger than your neighbors.
    function checks the ids of the current kilobot's neighbors.
*/
void check_closest_MIS()
{
    uint8_t index = 0;
    while (index < mydata->num_neighbors && !mydata->not_in_mis)
    {
        if (mydata->my_id > mydata->nearest_neighbors[index].id)
        {
            mydata->not_in_mis = 2;
        }
        if (mydata->nearest_neighbors[index].in_mis)
        {
            mydata->not_in_mis = 1;
        }
        else
            ++index;
    }
    if (!mydata->not_in_mis)
        mydata->in_mis = 1;
}
/**
 * Main function for setting up the checking functions for neighbors for the MIS. 
 * If not_in_mis == 2 then it means that the current kilobot did not enter the mis because its id is bigger than all of its neighbors.
 **/
void check_neighbors()
{
    if (mydata->round_counter >= 100)
    {
        mydata->round_counter = 0;
        if (mydata->num_neighbors > 0)
        {
            check_closest_MIS();
            if (mydata->not_in_mis == 2)
                check_if_no_MIS_nearby();
            if (mydata->in_mis)
            {
                mydata->red = 255;
                mydata->blue = 0;
                mydata->green = 0;
            }
        }
    }
    else
    {
        ++mydata->round_counter;
    }       
}

/**
* Checks if there's a conflict between neighbors. If so then reset the kilobots and try the algorithm again.
**/
void check_conflict()
{
    uint8_t index = 0;
    while (index < mydata->num_neighbors)
    {
        if (mydata->in_mis && mydata->nearest_neighbors[index].in_mis)
        {
            reset_data();
            return;
        }
        ++index;
    }
}

/**
 * Modified loop which accounts for messages received.
 * If at least one message from each neighbor is not received in the 50 loop cycles, then this kilobot is reset.
 * Performs the 6 color id reduction and sets nodes to new colors.
 **/
void loop()
{
    delay(30);
    send_move();
    // send_joining();
    send_sharing();
    move(mydata->now);

    check_messages(); // Incase the kilobots get disconnected. Reset them.
    check_neighbors(); // MIS algorithm

    check_conflict();
    set_color(RGB(mydata->red, mydata->green, mydata->blue));
    mydata->now++;
}

message_t *message_tx()
{
    
    if (mydata->tail != mydata->head)   // Queue is not empty
    {
/*#ifdef SIMULATOR
    printf("%d, Sending  %d  %d\n", mydata->my_id, mydata->message[mydata->head].data[MSG] , mydata->message[mydata->head].data[RECEIVER]);
#endif*/
        return &mydata->message[mydata->head];
    }
    return &mydata->nullmessage;
}
 
void message_tx_success() {
    if (mydata->tail != mydata->head) {  // Queue is not empty
/*#ifdef SIMULATOR
        printf("%d Sent  %d,  %d\n", mydata->my_id, mydata->message[mydata->head].data[MSG], mydata->message[mydata->head].data[RECEIVER]);
#endif*/
        if (mydata->copies == 2)
        {
            mydata->head++;
            mydata->copies = 0;
            mydata->head = mydata->head % QUEUE;
        }
        else
        {
            mydata->copies++;
        }
    }
}

void setup() {
    rand_seed(rand_hard());
    mydata->my_id = kilo_uid;
    
    mydata->state = AUTONOMOUS;
    mydata->my_left = mydata->my_right = mydata->my_id;
    mydata->num_neighbors = 0;
    mydata->message_sent = 0,
    mydata->now = 0,
    mydata->nextShareSending = SHARING_TIME,
    mydata->cur_motion = STOP;
    mydata->motion_state = STOP;
    mydata->time_active = 0;
    mydata->move_state = 0;
    mydata->move_motion[0].motion = LEFT;
    mydata->move_motion[0].motion = 3;
    mydata->move_motion[1].motion = RIGHT;
    mydata->move_motion[1].motion = 5;
    mydata->move_motion[0].motion = LEFT;
    mydata->move_motion[0].motion = 2;
    mydata->red = 0,
    mydata->green = 0,
    mydata->blue = 0,
    mydata->send_token = 0;

    mydata->nullmessage.data[MSG] = NULL_MSG;
    mydata->nullmessage.crc = message_crc(&mydata->nullmessage);
    
    mydata->token = rand_soft() < 128  ? 1 : 0;
    mydata->blue = mydata->token;
    mydata->head = 0;
    mydata->tail = 0;
    mydata->copies = 0;

    mydata->round_counter = 0;
    mydata->loop_counter = 0;
    mydata->in_mis = 0;
    mydata->not_in_mis = 0;
   
#ifdef SIMULATOR
    printf("Initializing %d %d\n", mydata->my_id, mydata->token);
#endif

    mydata->message_sent = 1;
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
char *cb_botinfo(void)
{
    char *p = botinfo_buffer;
    p += sprintf (p, "ID: %d \n", kilo_uid);
    if (mydata->state == COOPERATIVE)
        p += sprintf (p, "State: COOPERATIVE\n");
    if (mydata->state == AUTONOMOUS)
        p += sprintf (p, "State: AUTONOMOUS\n");
    
    return botinfo_buffer;
}
#endif

int main() {
    kilo_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);
    
    return 0;
}
