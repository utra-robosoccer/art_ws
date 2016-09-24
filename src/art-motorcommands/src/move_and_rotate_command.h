#ifndef MOVE_AND_ROTATE_COMMAND_H
#define MOVE_AND_ROTATE_COMMAND_H

//Calculates all paths 
vector<Vector2> getPaths (const vector<geometry_msgs::Point32> & points)
{
    vector<Vector2> processedPath;

    int n = points.size ();
    if (n < 2)
    {
        cout << "Path is too short" << endl;
        return processedPath;
    }
    
    Vector2 point[n];
    
    for (int i = 0; i < n; i++)
    {
        geometry_msgs::Point32 p = points[i];

        int x = p.x;
        int y = p.y;
        
        point[i] = Vector2 (x, y);
    }
    Vector2 startPoint = point[0];
    int count = 0;

    // Sandro: I added this to make the deadSimpleCalculateCommand work.
    // If it messes up the more advanced stuff, we can kill it
    processedPath.push_back(startPoint);
    
    cout << point[0].x << " , " << point[0].y << endl;
    int newStartIndex = 0;
    for (int i=2; i<n; i++)
    {
        for (int j=newStartIndex; j<i; j++)
        {
            if (inRange (startPoint, point[j], point[i], RANGE) == false)   // safe RANGE has not been decided yet
            {
                startPoint = point[i-1];
                newStartIndex = i;
                cout << point[i-1].x << " , " << point[i-1].y << endl;
                processedPath.push_back (point[i-1]);
                break;
            }
        }
    }
    
    cout << point[n-1].x << " , " << point[n-1].y << endl;
    processedPath.push_back (point[n-1]);
    
    return processedPath;
}

template <size_t N>  vector<Vector2> getPaths (const int (&data)[N][2])
{
    vector<geometry_msgs::Point32> v;
    for (size_t i = 0; i < N; i++)
    {
        geometry_msgs::Point32 p;
        p.x = data[i][0];
        p.y = data[i][1];
        v.push_back(p);
    }
    
    geometry_msgs::Polygon poly;
    poly.points = v;

    vector<Vector2> processedPath;
    processedPath = getPaths (v);
    return processedPath;
}

vector<MotorCommand> deadSimpleCalculateCommand(vector<Vector2> processedPath)
{
    vector<MotorCommand> commandList;
    MotorCommand command;
    
    double t = 1.0;
    
    if (processedPath.size() < 2)
    {
        cout << "Oops, path is size " << processedPath.size() << ", too small!" << endl;
        command.leftSpeed = 0;
        command.rightSpeed = 0;
        command.timeInterval = t;
    }
    else
    {
        string dir = "";
        double deltaX = processedPath[1].x - processedPath[0].x;
        double deltaY = processedPath[1].y - processedPath[0].y;

        double angleR = atan2(deltaY, deltaX);
        double angleD = angleR * 180 / PI;

        if (angleD > 20)
        {
            //go left
            dir = "left";
            command.rightSpeed = SPEED;
            command.leftSpeed = 0.8*SPEED;
            command.timeInterval = t;
        }
        else if (angleD < -20)
        {
            //go right
            dir = "right";
            command.rightSpeed = 0.8*SPEED;
            command.leftSpeed = SPEED;
            command.timeInterval = t;
        }
        else
        {
            // go straight
            dir = "straight";
            command.rightSpeed = SPEED;
            command.leftSpeed = SPEED;
            command.timeInterval = t;
        }

        cout << "dx = " << deltaX
             << ", dy = " << deltaY
             << ", angleR = " << angleR
             << ", angleD = " << angleD
             << ", direction = " << dir
             << endl;
    }

    commandList.push_back(command);
    return commandList;
}

vector<MotorCommand> calculateCommand (vector<Vector2> processedPath)
{
    MotorCommand command = {0.0, 0.0, 0.0};
    vector<MotorCommand> commandList;
    double distance;
    double time;
    double angle;
    double omega;     // angular velocity
    Vector2 displacement_next;
    Vector2 displacement;
    
    int i;
    for (i=0; processedPath.size()>=2 && i<processedPath.size()-2; i++)
    {
        displacement_next = processedPath[i+2] - processedPath[i+1];
        displacement = processedPath[i+1] - processedPath[i];
        
        distance = displacement.Size ();    // return the magnitude of the displacement
        time = distance / SPEED;
        command.leftSpeed = SPEED;
        command.rightSpeed = SPEED;
        command.timeInterval = time;
        commandList.push_back (command);
        

        angle = GetAngle (displacement, displacement_next);
        omega = 2 * SPEED / WHEELGAP;
        time = angle / omega;
        
        if (angle >= 0)
        {
            command.leftSpeed = -SPEED;
            command.rightSpeed = SPEED;
            command.timeInterval = time;
        }
        else
        {
            command.leftSpeed = SPEED;
            command.rightSpeed = -SPEED;
            command.timeInterval = -time;
        }
        
        commandList.push_back (command);
    }
    
    displacement_next = processedPath[i+2] - processedPath[i+1];
    displacement = processedPath[i+1] - processedPath[i];
    
    distance = displacement.Size ();    // return the magnitude of the displacement
    time = distance / SPEED;
    command.leftSpeed = SPEED;
    command.rightSpeed = SPEED;
    command.timeInterval = time;
    commandList.push_back (command);
    return commandList;
}


pair<int, int> getCommand (float leftSpeed, float rightSpeed, vector<Vector2> path)
{
    pair<int, int> vec (1, 2);
    return vec;
}



#endif
