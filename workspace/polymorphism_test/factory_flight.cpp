#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

enum Seat {Economy, Premium, Business};

enum Airline {Delta, United, SouthWest};

// global map for querying
std::unordered_map<string, Airline> airlines{
    {"Delta", Delta},
    {"United", United},
    {"SouthWest", SouthWest}
};

std::unordered_map<string, Seat> seats{
    {"Economy", Economy},
    {"Premium", Premium},
    {"Business", Business}
};

struct Ticket
{
    Airline airline;
    Seat seat;
    float miles;
};


class AirlineCalculator{
    public:
        static AirlineCalculator* create(Airline airline);
        //virtual method to be override
        virtual float calc(const Ticket& ticket) const = 0;
    private:
        virtual ~AirlineCalculator() = default;

    //implementations
    protected:
        //common method: get oprating cost
        virtual float getOpCost (Ticket ticket) const{
            float op_cost = 0;
            switch (ticket.seat)
            {
            case (Economy):
                op_cost = getEconomy(ticket.dist);
            case (Premium):
                op_cost = getPremium(ticket.dist);
            case (Business):
                op_cost = getBusiness(ticket.dist);
            default:
                break;
            }

            return op_cost;
        }

        virtual float getEconomy(float d) const{
            return 0.0;
        }

        virtual float getPremium(float d) const{
            return 25.0;
        }

        virtual float getBusiness(float d) const{
            return 50.0 + 0.25*d;
        }
};

class DeltaCalc: public AirlineCalculator{
    public:
        float calc(const Ticket& ticket) const override{
            float op = getOpCost(ticket);

            return op + ticket.dist*0.5;
        }

        //Singleton
        static AirlineCalculator* instance(){
            static DeltaCalc calc;
            return &calc;
        }

        virtual ~DeltaCalc() = default;

    private:
        DeltaCalc() = default;
}

// Factory pattern
AirlineCalculator* AirlineCalculator::create(Airline airline){
    switch (airline)
    {
    case Delta:
        return DeltaCalc::instance();
    
    default:
        break;
    }
}

static Ticket parse_ticket(const string& s){
    // split by space
    std::vector<string> arr;
    std::stringstream ss(s);
    std::string token; //current reading

    while(std::getline(ss, token, ' ')){
        arr.push_back(token);
    }

    Ticket ticket;
    ticket.airline = airlines[arr[0]];
    ticket.seat = seats[arr[2]];
    ticket.dist = std::stof(arr[1]);

    return ticket;
}