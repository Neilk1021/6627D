class Vector2{
    private:
        double X;
        double Y;
    public:  
        bool operator<(Vector2 const &obj);
        bool operator>(Vector2 const &obj);
        Vector2 operator-(Vector2 const &obj);
        Vector2 operator+(Vector2 const &obj);
        static double DOT(Vector2, Vector2);
        static double MAG(Vector2);
        Vector2(double X, double Y);
        void Update(double, double);
        static double DOTRAD(Vector2, Vector2);
        static double DOTDEG(Vector2, Vector2);
        static int GetDirec(Vector2, Vector2);
};