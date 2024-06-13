FROM node:14 AS build
WORKDIR /app
COPY package.json .
RUN npm install
RUN npm install face-api.js
COPY . .
RUN npm run build

FROM nginx
COPY --from=build /app/build /usr/share/nginx/html