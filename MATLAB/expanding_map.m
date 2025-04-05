inputFilename = 'myMap.pgm';
outputFilename = 'expandedMap.pgm';

% خواندن تصویر PGM
img = imread(inputFilename);

% دریافت ابعاد تصویر فعلی
[rows, cols] = size(img);

% تعیین ابعاد هدف 
targetRows = 300;
targetCols = 300;

% بررسی اینکه آیا تصویر کوچک‌تر از ابعاد هدف است یا خیر
if rows > targetRows || cols > targetCols
    error('تصویر ورودی ابعاد بزرگتری نسبت به ابعاد هدف دارد.');
end

% محاسبه مقدار پد لازم برای هر طرف (برای قرارگیری مرکزی تصویر)
padTop = floor((targetRows - rows) / 2);
padBottom = ceil((targetRows - rows) / 2);
padLeft = floor((targetCols - cols) / 2);
padRight = ceil((targetCols - cols) / 2);

% پد کردن تصویر:
paddedImg = padarray(img, [padTop, padLeft], 0, 'pre');
paddedImg = padarray(paddedImg, [padBottom, padRight], 0, 'post');

% نمایش تصویر پد شده (اختیاری)
figure;
imshow(paddedImg, []);
title('تصویر پس از گسترش ابعاد به 400x400');

% ذخیره فایل جدید PGM
%imwrite(paddedImg, outputFilename);
fprintf('فایل گسترش‌یافته به نام %s ذخیره شد.\n', outputFilename);
