#!/bin/sh
touch gen.h
echo "#ifndef __GEN_H__" >> gen.h
echo "#define __GEN_H__" >> gen.h
echo "" >> gen.h
echo "#include <QObject>" >> gen.h
echo "#include <QWidget>" >> gen.h
echo "" >> gen.h
echo "class Gen : public QWidget {" >> gen.h
echo "    Q_OBJECT" >> gen.h
echo "public:" >> gen.h
echo "    explicit Gen(QWidget *parent = nullptr);" >> gen.h
echo "    virtual ~Gen();" >> gen.h
echo "signals:" >> gen.h
echo "public slots:" >> gen.h
echo "private slots:" >> gen.h
echo "public: // methods" >> gen.h
echo "private: // methods" >> gen.h
echo "public: // variables" >> gen.h
echo "private: // variables" >> gen.h
echo "};" >> gen.h
echo "" >> gen.h
echo "#endif /* __GEN_H__ */" >> gen.h