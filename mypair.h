// My pair
class myPair{
	public:
		bool operator<(const myPair& pp) const{
			if(this->hop == pp.hop) return this->dis > pp.dis;
			return this->hop > pp.hop;
		}
		int id;
		int hop;
		double dis;
};
