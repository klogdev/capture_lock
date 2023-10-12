#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

enum Seat {Economy, Premium, Business};

enum Airline {Delta, United, SouthWest,Luigi};

// global map for querying
std::unordered_map<std::string, Airline> airlines{
    {"Delta", Delta},
    {"United", United},
    {"SouthWest", SouthWest},
    {"Luigi", Luigi}
};

std::unordered_map<std::string, Seat> seats{
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

    //implementations
    protected:
        // derived class will inherits destructor by default
        virtual ~AirlineCalculator() = default;

        //common method: get oprating cost
        virtual float getOpCost (Ticket ticket) const{
            float op_cost = 0;
            switch (ticket.seat)
            {
            case (Economy):
                op_cost = getEconomy(ticket.miles);
                break;
            case (Premium):
                op_cost = getPremium(ticket.miles);
                break;
            case (Business):
                op_cost = getBusiness(ticket.miles);
                break;
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

            return op + ticket.miles*0.5;
        }

        // Singleton
        // initialization of local static variables 
        // is guaranteed to be thread-safe
        static AirlineCalculator* instance(){
            static DeltaCalc calc_ins;
            return &calc_ins;
        }

    private:
        DeltaCalc() = default;
};

class UnitedCalc: public AirlineCalculator{
    public:
        float calc(const Ticket& ticket) const override{
            float op = getOpCost(ticket);
            return op + ticket.miles*0.75;
        }

        static AirlineCalculator* instance(){
            static UnitedCalc calc_ins;
            return & calc_ins;
        }

    private:
        // ensure singleton pattern by init only from instance()
        UnitedCalc() = default;

    protected:
        // protected member can be overriden by the derived class method
        float getPremium(float d) const override{
            return 25.0f + 0.1*d;
        }
};

class SouthWestCalc:public AirlineCalculator{
    public:
        float calc(const Ticket& ticket) const override{
            return 1.0*ticket.miles;
        }

        static AirlineCalculator* instance(){
            static SouthWestCalc calc_ins;
            return &calc_ins;
        }

    private:
        SouthWestCalc() = default;
};

class LuigiCalc:public AirlineCalculator{
    public:
        float calc(const Ticket& ticket) const override{
            float op = getOpCost(ticket);
            return max(100.0f, 2*op);
        }
    static AirlineCalculator* instance(){
            static LuigiCalc calc_ins;
            return &calc_ins;
        }

    private:
        LuigiCalc() = default;
};

// Factory pattern
AirlineCalculator* AirlineCalculator::create(Airline airline){
    switch (airline)
    {
    case Delta:
        return DeltaCalc::instance();
    case United:
        return UnitedCalc::instance();
    case SouthWest:
        return SouthWestCalc::instance();
    case Luigi:
        return LuigiCalc::instance();
    default:
        throw std::runtime_error("Unknown airline");
    }
}

static Ticket parse_ticket(const std::string& s){
    // split by space
    std::vector<std::string> arr;
    std::stringstream ss(s);
    std::string token; //current reading

    while(std::getline(ss, token, ' ')){
        arr.push_back(token);
    }

    Ticket ticket;
    // Check and get the airline from the map
    auto it = airlines.find(arr[0]);
    if (it == airlines.end()) {
        throw std::runtime_error("Unknown airline: " + arr[0]);
    }
    ticket.airline = it->second;
    ticket.seat = seats[arr[2]];
    ticket.miles = std::stof(arr[1]);

    return ticket;
}

// process all tickets
void process_tickets(std::vector<std::string> tickets, 
                    std::vector<float>& costs){

    for(const std::string& tic_str: tickets){
        Ticket ticket = parse_ticket(tic_str);

        AirlineCalculator* curr_calc = AirlineCalculator::create(ticket.airline);
        costs.push_back(curr_calc->calc(ticket));
    }
}

int main(){
    std::vector<std::string> input{"United 150.0 Premium", "United 120.0 Economy","United 100.0 Business","Delta 60.0 Economy", "Delta 60.0 Premium","Delta 60.0 Business", "SouthWest 1000.0 Economy","SouthWest 4000.0 Economy","ANA 300.0 Economy"};

    std::vector<float> costs;
    process_tickets(input, costs);

    for(int i = 0; i < input.size(); i++){
        std::cout << input[i] << " cost: $" << costs[i] << std::endl;
    }

    return 0;
}